# ui_helper
import streamlit as st
# =========================
# help func
# =========================
def parse_csv_list(text):# 處理空
    return [s.strip() for s in text.split(",") if s.strip()]

def resize_list(data_list, target_size):# 整理清單

    while len(data_list) < target_size:
        data_list.append({})

    while len(data_list) > target_size:
        data_list.pop()


def node_selector(# 選點
    label,
    options,
    default=None,
    multi=False,
    max_select=None
):

    default = default or []

    if multi:

        selected = st.multiselect(
            label,
            options,
            default=[n for n in default if n in options]
        )

        # 限制數量（如果有設定）
        if max_select is not None and len(selected) > max_select:
            st.warning(f"最多只能選 {max_select} 個")
            selected = selected[:max_select]

        return selected

    else:

        if default not in options:
            default = options[0] if options else None

        return st.selectbox(
            label,
            options,
            index=options.index(default) if default in options else 0
        )

def edit_axis_nodes(label, current):# 軸點編輯(沒用到了)

    text = st.text_input(
        label,
        ",".join(current)
    )

    return parse_csv_list(text)


def render_vector_section(
    cfg,
    section_name,
    value_key,
    title,
    node_options
):

    st.header(title)

    cfg.setdefault(section_name, [])
    data = cfg[section_name]

    num_items = st.number_input(
        f"Number of {title}",
        min_value=0,
        value=len(data),
        step=1
    )

    resize_list(data, num_items)

    for i, item in enumerate(data):

        st.subheader(f"{title[:-1]} {i}")

        # =========================
        # NAME
        # =========================
        item["name"] = st.text_input(
            f"{section_name}_name_{i}",
            item.get("name", "")
        )

        # =========================
        # NODE (單選)
        # =========================
        item["node"] = node_selector(
            f"{section_name}_node_{i}",
            node_options,
            default=item.get("node", ""),
            multi=False
        )

        # =========================
        # AXIS NODES (多選)
        # =========================
        item["axis_nodes"] = node_selector(
            f"{section_name}_axis_{i}",
            node_options,
            default=item.get("axis_nodes", []),
            multi=True,
            max_select=2   # ← 如果你要限制兩個
        )

        # =========================
        # VALUE
        # =========================
        item[value_key] = st.number_input(
            f"{section_name}_{value_key}_{i}",
            value=float(item.get(value_key, 0.0))
        )

        st.divider()

# for inputs

def render_motion(i, motion):

    motion["motion_type"] = st.selectbox(
        f"motion_type_{i}",
        ["s", "r", "f"],
        index=["s", "r", "f"].index(
            motion.get("motion_type", "s")
        )
    )

    motion["input_type"] = st.selectbox(
        f"input_type_{i}",
        ["l", "h"],
        index=["l", "h"].index(
            motion.get("input_type", "l")
        )
    )

    motion["x_max"] = st.number_input(
        f"x_max_{i}",
        value=float(motion.get("x_max", 0))
    )

    motion["x_min"] = st.number_input(
        f"x_min_{i}",
        value=float(motion.get("x_min", 0))
    )

    motion["times"] = st.number_input(
        f"times_{i}",
        value=int(motion.get("times", 1))
    )

    return motion

def render_list_section(cfg, key, title, render_item_fn):

    st.header(title)

    cfg.setdefault(key, [])
    data = cfg[key]

    num_items = st.number_input(
        f"Number of {title}",
        min_value=0,
        value=len(data),
        step=1
    )

    resize_list(data, num_items)

    for i, item in enumerate(data):

        st.subheader(f"{title[:-1]} {i}")

        render_item_fn(i, item)

        st.divider()

def render_input_item(i, inp, node_options):

    inp["name"] = st.text_input(
        f"name_{i}",
        inp.get("name", "")
    )

    inp["node"] = node_selector(
        f"node_{i}",
        node_options,
        default=inp.get("node", ""),
        multi=False
    )

    inp["constraint"] = st.text_input(
        f"constraint_{i}",
        inp.get("constraint", "")
    )

    inp["axis_nodes"] = node_selector(
        f"axis_nodes_{i}",
        node_options,
        default=inp.get("axis_nodes", []),
        multi=True,
        max_select=2
    )

    inp.setdefault("motion", {})

    inp["motion"] = render_motion(i, inp["motion"])

    inp["enable"] = st.checkbox(
        f"enable_{i}",
        value=inp.get("enable", True)
    )


def render_force_section(cfg, node_options):

    st.header("Forces")

    cfg.setdefault("forces", [])
    data = cfg["forces"]

    # =========================
    # size control
    # =========================
    num_items = st.number_input(
        "Number of Forces",
        min_value=0,
        value=len(data),
        step=1
    )

    resize_list(data, num_items)

    # =========================
    # force editor
    # =========================
    for i, item in enumerate(data):

        st.subheader(f"Force {i}")

        item.setdefault("node", node_options[0])
        item.setdefault("value", [0.0, 0.0, 0.0])

        # =========================
        # node selector
        # =========================
        item["node"] = st.selectbox(
            f"force_node_{i}",
            node_options,
            index=node_options.index(item["node"]) if item["node"] in node_options else 0,
            key=f"force_node_{i}"
        )

        # =========================
        # VECTOR INPUT (XYZ unified style)
        # =========================
        col1, col2, col3 = st.columns(3)

        with col1:
            item["value"][0] = st.number_input(
                f"Fx {i}",
                value=float(item["value"][0]),
                key=f"force_fx_{i}"
            )

        with col2:
            item["value"][1] = st.number_input(
                f"Fy {i}",
                value=float(item["value"][1]),
                key=f"force_fy_{i}"
            )

        with col3:
            item["value"][2] = st.number_input(
                f"Fz {i}",
                value=float(item["value"][2]),
                key=f"force_fz_{i}"
            )

        st.divider()

