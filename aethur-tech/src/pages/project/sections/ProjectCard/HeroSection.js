import React from "react";
import { Box, Typography, Container, Chip, Grid } from "@mui/material";

export default function HeroSection({ heroImage }) {
  return (
    <Box
      sx={{
        backgroundColor: "black",
        paddingY: { xs: "2rem", md: "3rem" }, // Adjust vertical padding
        minHeight: "50vh", // Maintain good visibility
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
      }}
    >
      <Container maxWidth="lg">
        <Grid container spacing={3} alignItems="center">
          {/* Left Content Section */}
          <Grid item xs={12} md={6}>
            <Box display="flex" alignItems="center" gap={1} marginBottom={1}>
              <Chip
                label="🐶 AI-Powered Pet Training"
                sx={{
                  backgroundColor: "rgba(255, 138, 0, 0.15)",
                  color: "#FF861D",
                  fontWeight: "bold",
                }}
              />
              <Chip
                label="5 min read"
                sx={{
                  backgroundColor: "rgba(255, 255, 255, 0.1)",
                  color: "#aaa",
                }}
              />
            </Box>

            <Typography
              variant="h4"
              sx={{ fontWeight: "bold", color: "white", marginBottom: "0.5rem" }}
            >
              Meet TAMIR: The Future of Pet Behavior Monitoring
            </Typography>
            <Typography
              variant="body1"
              sx={{ color: "rgba(255, 255, 255, 0.8)", marginBottom: "0rem" }}
            >
              TAMIR is an intelligent robotic system that **monitors pets in real-time** using AI and ROS2, detects misbehavior, and issues corrective signals to reinforce good habits. Perfect for pet owners who want peace of mind when they’re away.
            </Typography>
          </Grid>

          {/* Right Image Section */}
          <Grid item xs={12} md={6}>
            <Box
              sx={{
                borderRadius: "12px",
                overflow: "hidden",
                boxShadow: "0px 4px 15px rgba(255, 190, 70, 0.2)",
                textAlign: "center",
              }}
            >
              <video
                src={heroImage}
                autoPlay
                loop
                muted
                style={{
                  width: "100%",
                  maxHeight: "600px", // Maintain balance
                  borderRadius: "12px",
                  objectFit: "cover",
                }}
              />
            </Box>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
}
