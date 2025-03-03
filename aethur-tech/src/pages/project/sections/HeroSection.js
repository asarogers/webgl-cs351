import React from "react";
import { Box, Container, Typography, Paper, Button, Stack } from "@mui/material";
import backgroundImage from "../imgs/hero-background.png"; // Replace with actual background image

const HeroSection = () => {
  return (
    <Box
      sx={{
        paddingTop: "70px",
        height: "40vh", // Adjust height to match the design
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        textAlign: "center",
        backgroundImage: `url(${backgroundImage})`,
        backgroundSize: "cover",
        backgroundPosition: "center",
        position: "relative",
        "&::before": {
          content: '""',
          position: "absolute",
          top: 0,
          left: 0,
          width: "100%",
          height: "100%",
          background: "rgba(0, 0, 0, 0.5)", // Dark overlay for contrast
          backdropFilter: "blur(3px)", // Slight blur effect
        },
      }}
    >
      <Container maxWidth="md" sx={{ position: "relative", zIndex: 2 }}>
        {/* Glassmorphism Styled Box */}
        <Paper
          sx={{
            padding: "2rem",
            borderRadius: "15px",
            background: "rgba(0, 0, 0, 0.5)", // Semi-transparent background
            backdropFilter: "blur(10px)", // Glassmorphism effect
            boxShadow: "0px 4px 20px rgba(255, 190, 70, 0.3)",
            textAlign: "center",
          }}
        >
          <Typography variant="h3" sx={{ fontWeight: "bold", color: "white" }}>
            Our Portfolio
          </Typography>
          <Typography variant="body1" sx={{ color: "rgba(255, 255, 255, 0.8)", marginTop: "0.5rem" }}>
            Leading the Way in Crypto with Vision and Dedication
          </Typography>

          
        </Paper>
      </Container>
    </Box>
  );
};

export default HeroSection;
