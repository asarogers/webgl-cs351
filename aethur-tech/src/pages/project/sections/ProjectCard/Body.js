import React from "react";
import {
  Box,
  Container,
  Typography,
  Button,
  Grid,
  Paper,
} from "@mui/material";
import { useParams, useNavigate } from "react-router-dom";
import Sidebar from "./Sidebar";
import project1 from "../../imgs/project1.png";
import project2 from "../../imgs/project2.png";
import project3 from "../../imgs/project3.png";

// Sample project data
const projectData = {
  1: {
    title: "Setting Up the ROSBOT 2R",
    img: project1,
    date: "March 24, 2024",
    author: "John Doe",
    category: "Robotics",
    introText: "The ROSBOT 2R is an autonomous mobile robot designed for research and development in robotics. This guide walks through the setup process and core concepts.",
    sections: [
      {
        subtitle: "Understanding ROSBOT 2R",
        content:
          "The ROSBOT 2R is equipped with LiDAR, an IMU, and wheel encoders for localization and navigation. It supports ROS 2 and can be programmed using Python and C++.",
        img: project2,
      },
      {
        subtitle: "Hardware Setup",
        content:
          "To set up the ROSBOT 2R, assemble the hardware components, connect the power supply, and ensure that the onboard computer is functioning correctly.",
        img: project3,
      },
    ],
  },
};

const Body = () => {
  const { id } = useParams();
  const navigate = useNavigate();
  const project = projectData[id];

  if (!project)
    return (
      <Typography
        variant="h5"
        sx={{ color: "red", textAlign: "center", marginTop: "2rem" }}
      >
        Project Not Found
      </Typography>
    );

  return (
    <Container maxWidth="lg" sx={{ color: "white", paddingTop: "0rem" }}>
      <Grid container spacing={2}>
        <Grid item xs={12} md={4}>
          <Sidebar />
        </Grid>

        <Grid item xs={12} md={8}>
          <Paper sx={{ padding: "2rem", backgroundColor: "#222", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "white" }}>Introduction</Typography>
            <Typography variant="body1" sx={{ color: "rgba(255,255,255,0.8)", marginBottom: "2rem" }}>{project.introText}</Typography>
            <img src={project.img} alt={project.title} style={{ width: "100%", borderRadius: "10px", marginBottom: "1.5rem" }} />
          </Paper>

          <Paper sx={{ padding: "2rem", backgroundColor: "#222", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "white" }}>Prerequisites</Typography>
            <Typography variant="body1" sx={{ color: "rgba(255,255,255,0.8)" }}>Ensure you have ROS 2 installed, a compatible Linux system, and basic knowledge of terminal commands.</Typography>
          </Paper>

          <Paper sx={{ padding: "2rem", backgroundColor: "#222", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "white" }}>Setup / Installation</Typography>
            <Typography variant="body1" sx={{ color: "rgba(255,255,255,0.8)" }}>Download the ROSBOT 2R firmware, configure ROS 2 workspace, and calibrate sensors for accurate localization.</Typography>
          </Paper>

          {project.sections.map((section, index) => (
            <Paper key={index} sx={{ padding: "2rem", backgroundColor: "#222", borderRadius: "10px", marginBottom: "2rem" }}>
              <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "white" }}>{section.subtitle}</Typography>
              <Typography variant="body1" sx={{ marginBottom: "1rem", color: "rgba(255,255,255,0.8)" }}>{section.content}</Typography>
              <img src={section.img} alt={section.subtitle} style={{ width: "100%", borderRadius: "10px" }} />
            </Paper>
          ))}

          <Paper sx={{ padding: "2rem", backgroundColor: "#222", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "white" }}>Conclusion</Typography>
            <Typography variant="body1" sx={{ color: "rgba(255,255,255,0.8)" }}>You have successfully set up the ROSBOT 2R and can now start running navigation tests.</Typography>
          </Paper>

          
        </Grid>
      </Grid>
    </Container>
  );
};

export default Body;
