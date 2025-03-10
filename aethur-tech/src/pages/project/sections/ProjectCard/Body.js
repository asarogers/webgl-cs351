import React, { useEffect, useState } from "react";
import {
  Container,
  Typography,
  Grid,
  Paper,
} from "@mui/material";
import { useParams } from "react-router-dom";
import Sidebar from "./Sidebar";
import phoneView from "../../imgs/TAMIR/phoneView.mp4";
import waveVideo from "../../imgs/TAMIR/waveVideo.webm";
import teleop from "../../imgs/TAMIR/teleop.mp4";
import firstMap from "../../imgs/TAMIR/firstMap.webm";

// API Endpoint for Real-Time Data
const API_URL = "http://localhost:5000/behavior";

// Updated Project Data
const projectData = {
  1: {
    title: "TAMIR: The Training Assistive Mobile Intelligent Robot",
    date: "March 24, 2024",
    author: "John Doe",
    category: "Robotics & AI",
    introText:
      "TAMIR is an autonomous mobile robot that monitors pets and corrects undesirable behavior when owners are away. Using ROS 2, SLAM, and YOLO-based computer vision, TAMIR tracks pets, detects misbehavior, and issues corrective signals.",
    sections: [
      {
        subtitle: "Understanding TAMIR",
        content:
          "TAMIR is built on the ROSBOT 2R and leverages Simultaneous Localization and Mapping (SLAM) to navigate a home autonomously. Equipped with a RealSense2 camera and a Raspberry Pi 4, it detects pets in real-time and prevents unwanted behaviors.",
        video: firstMap,
      },
      {
        subtitle: "Hardware & Software Used",
        content:
          "TAMIR runs on a ROS 2-enabled Raspberry Pi 4, using a RealSense2 camera for depth perception. It employs YOLO for object detection and behavior classification, combined with geofencing logic for intelligent behavior correction.",
        video: teleop,
      },
      {
        subtitle: "Behavior Monitoring & Correction",
        content:
          "Using YOLO-based pet detection, TAMIR identifies unwanted behaviors like jumping on counters or knocking over trash cans. It then issues corrective signals via sound emitters, helping reinforce positive pet behavior.",
        video: phoneView,
      },
      {
        subtitle: "Geofencing Implementation",
        content:
          "TAMIR maps the home using SLAM and sets virtual geofences in restricted areas. If a pet enters a no-go zone, the robot navigates to the area and provides corrective feedback in real-time.",
      },
    ],
  },
};

const Body = () => {
  const { id } = useParams();
  const project = projectData[id];

  // State for real-time behavior updates
  const [behaviorData, setBehaviorData] = useState({
    petDetected: false,
    behavior: "No Activity",
    correctiveSignal: false,
    petPosition: { x: 0, y: 0 },
    geofence: { x: 100, y: 100, width: 200, height: 200 },
  });

  // Fetch real-time behavior data
  useEffect(() => {
    const fetchBehaviorData = async () => {
      try {
        const response = await fetch(API_URL);
        const data = await response.json();
        setBehaviorData(data);
      } catch (error) {
        console.error("Error fetching behavior data:", error);
      }
    };

    // Polling every 2 seconds
    const interval = setInterval(fetchBehaviorData, 2000);
    return () => clearInterval(interval);
  }, []);

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
    <Container maxWidth="lg" sx={{ color: "#ddd", paddingTop: "1rem" }}>
      <Grid container spacing={2}>
        <Grid item xs={12} md={4}>
          <Sidebar />
        </Grid>

        <Grid item xs={12} md={8}>
          {/* Introduction */}
          <Paper sx={{ padding: "2rem", backgroundColor: "#333", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#FF861D" }}>
              Introduction
            </Typography>
            <Typography variant="body1" sx={{ color: "#ddd", marginBottom: "2rem" }}>
              {project.introText}
            </Typography>
            <video autoPlay loop muted src={waveVideo} 
              style={{ maxHeight: "500px", width: "100%", borderRadius: "10px", marginBottom: "1.5rem" }} />
          </Paper>

          {/* Live Detection Status */}
          <Paper sx={{ padding: "2rem", backgroundColor: "#444", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#FF861D" }}>
              Live Detection Status
            </Typography>
            <Typography variant="body1" sx={{ color: "#ddd" }}>
              <strong>Detected Pet:</strong> {behaviorData.petDetected ? "Dog" : "None"}
            </Typography>
            <Typography variant="body1" sx={{ color: "#ddd" }}>
              <strong>Current Behavior:</strong> {behaviorData.behavior}
            </Typography>
            <Typography variant="body1" sx={{ color: "#ddd" }}>
              <strong>Corrective Signal Sent:</strong> {behaviorData.correctiveSignal ? "Yes" : "No"}
            </Typography>
          </Paper>

          {/* Project Sections */}
          {project.sections.map((section, index) => (
            <Paper key={index} sx={{ padding: "2rem", backgroundColor: index % 2 === 0 ? "#333" : "#444", borderRadius: "10px", marginBottom: "2rem" }}>
              <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#FF861D" }}>
                {section.subtitle}
              </Typography>
              <Typography variant="body1" sx={{ marginBottom: "1rem", color: "#ddd" }}>
                {section.content}
              </Typography>
              {section.video && (
                <video controls src={section.video} 
                  style={{ maxHeight: "500px",width: "100%", borderRadius: "10px", marginTop: "1rem" }} />
              )}
            </Paper>
          ))}

          {/* Conclusion */}
          <Paper sx={{ padding: "2rem", backgroundColor: "#333", borderRadius: "10px", marginBottom: "2rem" }}>
            <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#FF861D" }}>
              Conclusion
            </Typography>
            <Typography variant="body1" sx={{ color: "#ddd" }}>
              TAMIR successfully maps homes, follows pets, detects and corrects behavior using ROS 2 and AI-based vision models, providing a robust pet monitoring solution.
            </Typography>
          </Paper>
        </Grid>
      </Grid>
    </Container>
  );
};

export default Body;
