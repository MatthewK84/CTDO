"""
AFCENT CTDO Drone Component Laydown Manager
A Streamlit application for managing drone inventory with:
- Multi-sheet Excel/CSV viewing
- Low quantity highlighting (red for < 5)
- Image storage per item
- Order request generation for critical items
- File upload to refresh data
"""

import streamlit as st
import pandas as pd
import sqlite3
import base64
import io
import os
from datetime import datetime
from pathlib import Path

# Page configuration
st.set_page_config(
    page_title="AFCENT CTDO Drone Inventory",
    page_icon="🛸",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Database setup
DB_PATH = "inventory_images.db"

def init_db():
    """Initialize SQLite database for image storage"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS item_images (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            sheet_name TEXT,
            item_name TEXT,
            image_data BLOB,
            image_name TEXT,
            uploaded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            UNIQUE(sheet_name, item_name)
        )
    ''')
    c.execute('''
        CREATE TABLE IF NOT EXISTS order_requests (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            sheet_name TEXT,
            item_name TEXT,
            quantity_requested INTEGER,
            priority TEXT,
            notes TEXT,
            status TEXT DEFAULT 'Pending',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    conn.commit()
    conn.close()

def save_image(sheet_name: str, item_name: str, image_data: bytes, image_name: str):
    """Save or update an image for an item"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''
        INSERT OR REPLACE INTO item_images (sheet_name, item_name, image_data, image_name, uploaded_at)
        VALUES (?, ?, ?, ?, ?)
    ''', (sheet_name, item_name, image_data, image_name, datetime.now()))
    conn.commit()
    conn.close()

def get_image(sheet_name: str, item_name: str):
    """Retrieve an image for an item"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('SELECT image_data, image_name FROM item_images WHERE sheet_name = ? AND item_name = ?',
              (sheet_name, item_name))
    result = c.fetchone()
    conn.close()
    return result

def get_all_images_for_sheet(sheet_name: str):
    """Get all items with images for a sheet"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('SELECT item_name FROM item_images WHERE sheet_name = ?', (sheet_name,))
    results = [row[0] for row in c.fetchall()]
    conn.close()
    return results

def save_order_request(sheet_name: str, item_name: str, qty: int, priority: str, notes: str):
    """Save an order request"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''
        INSERT INTO order_requests (sheet_name, item_name, quantity_requested, priority, notes)
        VALUES (?, ?, ?, ?, ?)
    ''', (sheet_name, item_name, qty, priority, notes))
    conn.commit()
    conn.close()

def get_order_requests():
    """Get all order requests"""
    conn = sqlite3.connect(DB_PATH)
    df = pd.read_sql_query('SELECT * FROM order_requests ORDER BY created_at DESC', conn)
    conn.close()
    return df

def update_order_status(order_id: int, new_status: str):
    """Update order request status"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('UPDATE order_requests SET status = ? WHERE id = ?', (new_status, order_id))
    conn.commit()
    conn.close()

def delete_order_request(order_id: int):
    """Delete an order request"""
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('DELETE FROM order_requests WHERE id = ?', (order_id,))
    conn.commit()
    conn.close()

# Mapping of sheets to their quantity columns
QUANTITY_COLUMNS = {
    'On Hand Drone Parts List': 'Quantity',
    'On Hand Battery Inventory': 'On-Hand',
    'Trapper Part List': 'Quantity',
    'SICA BTU BOM': 'Count per aircraft',
    'SICA Tools List': 'GAP',
    'REDDI Part List': 'Quantity',
    'BlackBird Part List': 'Gap',
    'Open Purchase Request 1': 'QTY',
    'Open Purchase Request 2': 'Quantity'
}

# Mapping of sheets to their item name columns
ITEM_COLUMNS = {
    'On Hand Drone Parts List': 'Item Name',
    'On Hand Battery Inventory': 'Vendor',
    'Trapper Part List': 'Item',
    'SICA BTU BOM': 'Component',
    'SICA Tools List': 'Assembly Tools',
    'REDDI Part List': 'Component',
    'BlackBird Part List': 'Blackbird Components ',
    'Open Purchase Request 1': 'Drone Items',
    'Open Purchase Request 2': 'Item'
}

def get_quantity_column(sheet_name: str) -> str:
    """Get the quantity column name for a sheet"""
    return QUANTITY_COLUMNS.get(sheet_name, None)

def get_item_column(sheet_name: str) -> str:
    """Get the item name column for a sheet"""
    return ITEM_COLUMNS.get(sheet_name, None)

def highlight_low_quantity(df: pd.DataFrame, qty_col: str):
    """Apply red highlighting to rows with quantity < 5"""
    if qty_col and qty_col in df.columns:
        def highlight_row(row):
            try:
                qty = pd.to_numeric(row[qty_col], errors='coerce')
                if pd.notna(qty) and qty < 5:
                    return ['background-color: #ffcccc; color: #8b0000'] * len(row)
            except:
                pass
            return [''] * len(row)
        return df.style.apply(highlight_row, axis=1)
    return df.style

def make_clickable(val):
    """Make URLs clickable in dataframe"""
    if pd.isna(val):
        return val
    if isinstance(val, str) and ('http://' in val or 'https://' in val):
        # Truncate display but keep full URL
        display = val[:50] + '...' if len(val) > 50 else val
        return f'<a href="{val}" target="_blank">{display}</a>'
    return val

def load_excel_data(file_path_or_buffer):
    """Load all sheets from Excel file"""
    try:
        xl = pd.ExcelFile(file_path_or_buffer)
        data = {}
        for sheet in xl.sheet_names:
            df = pd.read_excel(xl, sheet_name=sheet)
            data[sheet] = df
        return data
    except Exception as e:
        st.error(f"Error loading file: {e}")
        return None

def get_low_stock_items(data: dict) -> pd.DataFrame:
    """Get all items with quantity 0 or 1 across all sheets"""
    low_stock = []
    for sheet_name, df in data.items():
        qty_col = get_quantity_column(sheet_name)
        item_col = get_item_column(sheet_name)
        
        if qty_col and qty_col in df.columns and item_col and item_col in df.columns:
            for idx, row in df.iterrows():
                try:
                    qty = pd.to_numeric(row[qty_col], errors='coerce')
                    if pd.notna(qty) and qty <= 1:
                        # Try to get link if available
                        link = None
                        for link_col in ['Link', 'PURCHASE LINK', 'Purchase Links']:
                            if link_col in df.columns and pd.notna(row.get(link_col)):
                                link = row[link_col]
                                break
                        
                        low_stock.append({
                            'Sheet': sheet_name,
                            'Item': row[item_col],
                            'Current Qty': int(qty) if pd.notna(qty) else 0,
                            'Link': link
                        })
                except:
                    pass
    
    return pd.DataFrame(low_stock)

# Initialize database
init_db()

# Custom CSS
st.markdown("""
<style>
    .stTabs [data-baseweb="tab-list"] {
        gap: 8px;
    }
    .stTabs [data-baseweb="tab"] {
        padding: 8px 16px;
        border-radius: 4px;
    }
    .low-stock-warning {
        background-color: #fff3cd;
        border: 1px solid #ffc107;
        border-radius: 8px;
        padding: 16px;
        margin: 10px 0;
    }
    .critical-stock {
        background-color: #f8d7da;
        border: 1px solid #dc3545;
        border-radius: 8px;
        padding: 16px;
        margin: 10px 0;
    }
    .metric-card {
        background-color: #f8f9fa;
        border-radius: 8px;
        padding: 16px;
        text-align: center;
    }
    div[data-testid="stDataFrame"] {
        width: 100%;
    }
</style>
""", unsafe_allow_html=True)

# Sidebar
with st.sidebar:
    st.image("https://upload.wikimedia.org/wikipedia/commons/thumb/6/69/USAF_logo.png/200px-USAF_logo.png", width=100)
    st.title("📦 Inventory Manager")
    st.markdown("---")
    
    # File upload section
    st.subheader("📤 Update Data")
    uploaded_file = st.file_uploader(
        "Upload new inventory file",
        type=['xlsx', 'xls', 'csv'],
        help="Upload a new Excel or CSV file to replace current inventory data"
    )
    
    if uploaded_file is not None:
        if st.button("🔄 Load New Data", type="primary"):
            st.session_state['uploaded_data'] = uploaded_file
            st.success("✅ Data loaded successfully!")
            st.rerun()
    
    st.markdown("---")
    
    # Navigation
    st.subheader("🧭 Navigation")
    page = st.radio(
        "Select View",
        ["📊 Inventory Dashboard", "🖼️ Image Manager", "📝 Order Requests"],
        label_visibility="collapsed"
    )

# Load data
@st.cache_data
def load_default_data():
    # For Streamlit Cloud: file is in same directory as app.py
    default_path = Path(__file__).parent / "AFCENT_CTDO_Drone_Component_Laydown.xlsx"
    if default_path.exists():
        return load_excel_data(default_path)
    return None

# Check for uploaded data or use default
if 'uploaded_data' in st.session_state:
    data = load_excel_data(st.session_state['uploaded_data'])
else:
    data = load_default_data()

if data is None:
    st.error("⚠️ No data available. Please upload an inventory file.")
    st.stop()

# Main content based on selected page
if page == "📊 Inventory Dashboard":
    st.title("🛸 AFCENT CTDO Drone Component Laydown")
    st.markdown("Real-time inventory tracking with low-stock alerts")
    
    # Summary metrics
    col1, col2, col3, col4 = st.columns(4)
    
    total_items = sum(len(df) for df in data.values())
    low_stock_df = get_low_stock_items(data)
    critical_count = len(low_stock_df[low_stock_df['Current Qty'] == 0])
    low_count = len(low_stock_df[low_stock_df['Current Qty'] == 1])
    
    with col1:
        st.metric("📋 Total Sheets", len(data))
    with col2:
        st.metric("📦 Total Line Items", total_items)
    with col3:
        st.metric("⚠️ Low Stock (Qty 1)", low_count)
    with col4:
        st.metric("🚨 Critical (Qty 0)", critical_count)
    
    st.markdown("---")
    
    # Alert banner for critical items
    if critical_count > 0:
        with st.expander(f"🚨 **CRITICAL: {critical_count} items at ZERO quantity** - Click to view", expanded=True):
            critical_items = low_stock_df[low_stock_df['Current Qty'] == 0]
            st.dataframe(critical_items, use_container_width=True, hide_index=True)
    
    # Tabs for each sheet
    tabs = st.tabs(list(data.keys()))
    
    for tab, (sheet_name, df) in zip(tabs, data.items()):
        with tab:
            st.subheader(f"📄 {sheet_name}")
            
            # Sheet stats
            qty_col = get_quantity_column(sheet_name)
            if qty_col and qty_col in df.columns:
                low_qty_count = len(df[pd.to_numeric(df[qty_col], errors='coerce') < 5].dropna(subset=[qty_col]))
                st.caption(f"📊 {len(df)} items | ⚠️ {low_qty_count} items with quantity < 5 (highlighted in red)")
            else:
                st.caption(f"📊 {len(df)} items")
            
            # Search/filter
            search = st.text_input(f"🔍 Search {sheet_name}", key=f"search_{sheet_name}")
            
            display_df = df.copy()
            
            # Apply search filter
            if search:
                mask = display_df.astype(str).apply(lambda x: x.str.contains(search, case=False, na=False)).any(axis=1)
                display_df = display_df[mask]
            
            # Display with highlighting
            if qty_col and qty_col in display_df.columns:
                styled_df = highlight_low_quantity(display_df, qty_col)
                st.dataframe(styled_df, use_container_width=True, height=400)
            else:
                st.dataframe(display_df, use_container_width=True, height=400)
            
            # Download button for this sheet
            csv = display_df.to_csv(index=False)
            st.download_button(
                label=f"📥 Download {sheet_name} as CSV",
                data=csv,
                file_name=f"{sheet_name.replace(' ', '_')}.csv",
                mime="text/csv",
                key=f"dl_{sheet_name}"
            )

elif page == "🖼️ Image Manager":
    st.title("🖼️ Item Image Manager")
    st.markdown("Upload and manage product images for inventory items")
    
    # Sheet selector
    sheet_name = st.selectbox("Select Sheet", list(data.keys()))
    df = data[sheet_name]
    item_col = get_item_column(sheet_name)
    
    if item_col and item_col in df.columns:
        # Get list of items
        items = df[item_col].dropna().unique().tolist()
        
        col1, col2 = st.columns([1, 2])
        
        with col1:
            st.subheader("📤 Upload Image")
            selected_item = st.selectbox("Select Item", items)
            
            uploaded_image = st.file_uploader(
                "Choose an image",
                type=['png', 'jpg', 'jpeg', 'gif', 'webp'],
                key=f"img_upload_{sheet_name}"
            )
            
            if uploaded_image is not None:
                st.image(uploaded_image, caption="Preview", width=200)
                if st.button("💾 Save Image", type="primary"):
                    image_data = uploaded_image.read()
                    save_image(sheet_name, selected_item, image_data, uploaded_image.name)
                    st.success(f"✅ Image saved for {selected_item}")
                    st.rerun()
        
        with col2:
            st.subheader("📸 Stored Images")
            items_with_images = get_all_images_for_sheet(sheet_name)
            
            if items_with_images:
                # Display images in a grid
                cols = st.columns(3)
                for idx, item_name in enumerate(items_with_images):
                    img_data = get_image(sheet_name, item_name)
                    if img_data:
                        with cols[idx % 3]:
                            st.image(img_data[0], caption=item_name[:30], use_container_width=True)
            else:
                st.info("No images uploaded for this sheet yet.")
    else:
        st.warning("Could not identify item column for this sheet.")

elif page == "📝 Order Requests":
    st.title("📝 Order Request Generator")
    st.markdown("Create and manage purchase requests for low-stock items")
    
    tab1, tab2 = st.tabs(["➕ Create New Request", "📋 View All Requests"])
    
    with tab1:
        # Get low stock items
        low_stock_df = get_low_stock_items(data)
        
        if len(low_stock_df) > 0:
            st.subheader("⚠️ Items Needing Reorder (Qty ≤ 1)")
            st.dataframe(low_stock_df, use_container_width=True, hide_index=True)
            
            st.markdown("---")
            st.subheader("Create Order Request")
            
            col1, col2 = st.columns(2)
            
            with col1:
                # Create combined item list for selection
                item_options = [f"{row['Sheet']} | {row['Item']}" for _, row in low_stock_df.iterrows()]
                selected = st.selectbox("Select Item to Order", item_options)
                
                if selected:
                    sheet, item = selected.split(" | ", 1)
                    item_row = low_stock_df[(low_stock_df['Sheet'] == sheet) & (low_stock_df['Item'] == item)].iloc[0]
                    
                    st.info(f"Current Quantity: **{item_row['Current Qty']}**")
                    
                    if pd.notna(item_row['Link']):
                        st.markdown(f"🔗 [Purchase Link]({item_row['Link']})")
                    
                    # Show image if available
                    img_data = get_image(sheet, item)
                    if img_data:
                        st.image(img_data[0], caption="Product Image", width=150)
            
            with col2:
                qty_to_order = st.number_input("Quantity to Order", min_value=1, value=5)
                priority = st.selectbox("Priority", ["🔴 Critical", "🟡 High", "🟢 Normal"])
                notes = st.text_area("Notes / Justification")
                
                if st.button("📝 Create Order Request", type="primary"):
                    if selected:
                        sheet, item = selected.split(" | ", 1)
                        save_order_request(sheet, item, qty_to_order, priority, notes)
                        st.success(f"✅ Order request created for {item}")
                        st.balloons()
        else:
            st.success("🎉 All items are well-stocked! No items with quantity ≤ 1.")
    
    with tab2:
        st.subheader("📋 All Order Requests")
        
        orders_df = get_order_requests()
        
        if len(orders_df) > 0:
            # Filter by status
            status_filter = st.multiselect(
                "Filter by Status",
                ["Pending", "Approved", "Ordered", "Received", "Cancelled"],
                default=["Pending", "Approved", "Ordered"]
            )
            
            filtered_orders = orders_df[orders_df['status'].isin(status_filter)]
            
            for idx, order in filtered_orders.iterrows():
                with st.expander(f"**{order['item_name']}** - {order['priority']} - {order['status']}"):
                    col1, col2, col3 = st.columns([2, 1, 1])
                    
                    with col1:
                        st.write(f"**Sheet:** {order['sheet_name']}")
                        st.write(f"**Quantity:** {order['quantity_requested']}")
                        st.write(f"**Notes:** {order['notes']}")
                        st.write(f"**Created:** {order['created_at']}")
                    
                    with col2:
                        new_status = st.selectbox(
                            "Update Status",
                            ["Pending", "Approved", "Ordered", "Received", "Cancelled"],
                            index=["Pending", "Approved", "Ordered", "Received", "Cancelled"].index(order['status']),
                            key=f"status_{order['id']}"
                        )
                        if st.button("Update", key=f"update_{order['id']}"):
                            update_order_status(order['id'], new_status)
                            st.success("Status updated!")
                            st.rerun()
                    
                    with col3:
                        if st.button("🗑️ Delete", key=f"delete_{order['id']}"):
                            delete_order_request(order['id'])
                            st.warning("Order deleted")
                            st.rerun()
            
            # Export orders
            st.markdown("---")
            csv = filtered_orders.to_csv(index=False)
            st.download_button(
                label="📥 Export Orders to CSV",
                data=csv,
                file_name=f"order_requests_{datetime.now().strftime('%Y%m%d')}.csv",
                mime="text/csv"
            )
        else:
            st.info("No order requests yet. Create one from the 'Create New Request' tab.")

# Footer
st.markdown("---")
st.caption("AFCENT CTDO Drone Component Laydown Manager | Built with Streamlit")
