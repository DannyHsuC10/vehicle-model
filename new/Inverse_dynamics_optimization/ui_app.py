import streamlit as st
from pathlib import Path
from Model_Analysis.UI import ui_helper as uih
from Model_Analysis.UI import ui_loader as uil
BASE_DIR = Path(__file__).resolve().parent

CONFIG_PATH = BASE_DIR/ "data"  / "test_set.json"
nodes_csv_path = BASE_DIR / "data" / "nodes.csv"
rod_csv_path = BASE_DIR / "data" / "rods.csv"

cfg = uil.load_json(CONFIG_PATH)

st.title("Mechanical Analysis Simulation seting")
rod_options = uil.load_rod_names(rod_csv_path)
node_options = uil.load_node_names(nodes_csv_path)

# =========================
# SIMULATION
# =========================
st.header("Simulation")

cfg.setdefault("simulation", {})

cfg["simulation"]["SIM_TIME"] = st.number_input(
    "SIM_TIME",
    min_value=0.1,
    max_value=50.0,   # ✅ 限制在 50
    value=float(cfg["simulation"].get("SIM_TIME", 10.0))
)

cfg["simulation"]["DT"] = st.number_input(
    "DT",
    0.001, 0.5,
    float(cfg["simulation"].get("DT", 0.05))
)

cfg["simulation"]["enable_visual"] = st.checkbox(
    "Enable Visual",
    cfg["simulation"].get("enable_visual", True)
)


# =========================
# FILES
# =========================
st.header("Files")

cfg.setdefault("files", {})

cfg["files"]["nodes_path"] = st.text_input(
    "Nodes Path",
    cfg["files"].get("nodes_path", "data/nodes.csv")
)

cfg["files"]["rods_path"] = st.text_input(
    "Rods Path",
    cfg["files"].get("rods_path", "data/rods.csv")
)


# =========================
# GEOMETRY OVERRIDE
# =========================
st.header("Geometry Override")

cfg.setdefault("geometry_override", {})

cfg["geometry_override"]["enable"] = st.checkbox(
    "Enable Geometry Override",
    cfg["geometry_override"].get("enable", False)
)

cfg["geometry_override"].setdefault("nodes", {})

if cfg["geometry_override"]["enable"]:
    for name, val in cfg["geometry_override"]["nodes"].items():
        st.subheader(f"Node: {name}")
        x = st.number_input(f"{name}_x", value=float(val[0]))
        y = st.number_input(f"{name}_y", value=float(val[1]))
        z = st.number_input(f"{name}_z", value=float(val[2]))
        cfg["geometry_override"]["nodes"][name] = [x, y, z]

# =========================
# INPUTS
# =========================

st.header("Inputs")

cfg.setdefault("inputs", [])

node_options = uil.load_node_names(nodes_csv_path)

uih.render_list_section(
    cfg,
    "inputs",
    "Inputs",
    lambda i, inp: uih.render_input_item(i, inp, node_options)
)

# =========================
# FORCES
# =========================
uih.render_vector_section(
    cfg,
    section_name="forces",
    value_key="force",
    title="Forces 力量設定",
    node_options = node_options
)
# =========================
# TORQUES
# =========================
uih.render_vector_section(
    cfg,
    section_name="torques",
    value_key="torque",
    title="Torques 力矩設定",
    node_options = node_options
)
# =========================
# TARGETS
# =========================
st.header("Targets")

cfg.setdefault("targets", {})

cfg["targets"]["node_targets"] = uih.node_selector(
    "node_targets",
    node_options,
    default=cfg["targets"].get("node_targets", []),
    multi=True
)

cfg["targets"]["rod_targets"] = uih.node_selector(
    "rod_targets",
    rod_options,
    default=cfg["targets"].get("rod_targets", []),
    multi=True
)

# =======================================================================================================
# STATIC
# =========================
st.header("Static Analysis")

cfg.setdefault("static", {})
static = cfg["static"]

# =========================
# STATIC FILES
# =========================
st.subheader("Files")

static.setdefault("files", {})

static["files"]["nodes_path"] = st.text_input(
    "Nodes Path (static)",
    static["files"].get("nodes_path", "data/nodes.csv"),
    key="static_nodes_path"
)

static["files"]["rods_path"] = st.text_input(
    "Rods Path (static)",
    static["files"].get("rods_path", "data/rods.csv"),
    key="static_rods_path"
)


# =========================
# FORCE (STATIC)
# =========================
uih.render_force_section(static, node_options)


# =========================
# STATIC OUTPUT
# =========================
st.subheader("Output")

static.setdefault("output", {})

static["output"]["report"] = st.checkbox(
    "Generate Report",
    static["output"].get("report", True)
)

static["output"]["csv"] = st.checkbox(
    "Save CSV",
    static["output"].get("csv", True)
)
# =============================================================================================================
# OUTPUT
# =========================
st.header("Output")

cfg.setdefault("output", {})

cfg["output"]["plot"] = st.text_input(# 繪圖
    "plot",
    ",".join(cfg["output"].get("plot", []))
).split(",")

cfg["output"]["save_plot"] = st.checkbox(
    "save_plot",
    cfg["output"].get("save_plot", True)
)

cfg.setdefault("output", {})# 印出點

cfg["output"]["print"] = uih.node_selector(
    "print outputs",
    node_options,
    default=cfg["output"].get("print", []),
    multi=True
)

cfg["output"]["save_csv"] = st.checkbox(# 存csv
    "save_csv",
    cfg["output"].get("save_csv", True)
)


# =========================
# SAVE
# =========================
if st.button("💾 Save JSON"):# 存檔案
    uil.save_json(cfg,CONFIG_PATH)
    st.success("Saved steer_set.json")

# =========================
# PREVIEW
# =========================
st.subheader("JSON Preview")
st.json(cfg)

