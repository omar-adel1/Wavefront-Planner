import streamlit as st
import numpy as np
from functions import load_mat_file, planner, plot_map, plot_trajectory

st.title("Wavefront Path Planner 🚀")

st.sidebar.header("Upload & Input Parameters")

uploaded_file = st.sidebar.file_uploader("Upload a .mat file", type=["mat"])

if uploaded_file:
    map_data = load_mat_file(uploaded_file)

    st.sidebar.subheader("Choose Start Position")
    start_row = st.sidebar.number_input("Enter the starting Y coordinate:", min_value=0, max_value=map_data.shape[0]-1, value=0)
    start_col = st.sidebar.number_input("Enter the starting X coordinate:", min_value=0, max_value=map_data.shape[1]-1, value=0)

    # Run Planner Button in Sidebar
    if st.sidebar.button("Run Planner"):
        try:
            # Compute wavefront value map and trajectory
            value_map, trajectory = planner(map_data.tolist(), start_row, start_col)

            col1, col2 = st.columns(2)

            # Original Map
            with col1:
                st.write("### Original Map")
                st.pyplot(plot_map(map_data, "Original Environment"))

            # Map with Trajectory
            with col2:
                st.write("### Wavefront Value Map with Trajectory")
                st.pyplot(plot_trajectory(value_map, trajectory))

            # Value Map 
            st.write("### Value Map (Numerical Representation)")
            st.text(np.array(value_map)) 

            # Trajectory
            st.write("### trajectory =")
            st.write(trajectory)

        except ValueError as e:
            st.error(f"Error: {e}")
