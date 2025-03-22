import streamlit as st
import numpy as np
import time 
import Wavefront_Path_functions as functions 
import os

st.title("Wavefront Path Planner 🚀")

st.sidebar.header("Upload & Input Parameters")

uploaded_file = st.sidebar.file_uploader("Upload a .mat file", type=["mat"])

if uploaded_file:
    map_data = functions.load_mat_file(uploaded_file)

    st.sidebar.subheader("Choose Start Position")
    start_row = st.sidebar.number_input("Enter the starting Y coordinate:", min_value=0, max_value=map_data.shape[0]-1, value=0)
    start_col = st.sidebar.number_input("Enter the starting X coordinate:", min_value=0, max_value=map_data.shape[1]-1, value=0)

    if st.sidebar.button("Run Planner"):
        try:
            start_time = time.time()
            value_map, trajectory = functions.planner(map_data.tolist(), start_row, start_col)

            end_time = time.time()  
            execution_time = end_time - start_time  
            col1, col2 = st.columns(2)

            # Original Map
            with col1:
                st.write("### Original Map")
                st.pyplot(functions.plot_map(map_data, "Original Environment"))

            # Map with Trajectory
            with col2:
                st.write("### Wavefront Value Map with Trajectory")
                st.pyplot(functions.plot_trajectory(value_map, trajectory))

            st.write(f"### Execution Time: {execution_time:.4f} seconds")
            # Value Map 
            st.write("### Value Map =")
            st.write(np.array(value_map))

            # Trajectory
            trajectory_text = functions.get_trajectory_text(trajectory)
            
            traj_col, download_col = st.columns([3, 1])
            with traj_col:
                st.write("### Trajectory")
                st.write(trajectory)
            
            filename = os.path.splitext(uploaded_file.name)[0]
            download_filename = f"{filename} trajectory.txt"
            
            with download_col:
                st.download_button(
                    label="Download Trajectory",
                    data=trajectory_text,
                    file_name=download_filename,
                    mime="text/plain"
                )

        except ValueError as e:
            st.error(f"Error: {e}")
