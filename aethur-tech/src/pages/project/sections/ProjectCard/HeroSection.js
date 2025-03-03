import React from "react";
import { Box, Typography, Container, Button, Chip, Grid } from "@mui/material";


export default function HeroSection({heroImage}) {
  return (
    <Box
      sx={{
        backgroundColor: "black",
        padding: { xs: "0rem", md: "0rem" },
        minHeight: "32vh",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
      }}
    >
      <Container maxWidth="lg">
        <Grid container spacing={1} alignItems="center">
          {/* Left Content Section */}
          <Grid item xs={12} md={6}>
            <Box display="flex" alignItems="center" gap={1} marginBottom={2}>
              <Chip label="🟠  Product" sx={{ backgroundColor: "rgba(255, 138, 0, 0.15)", color: "#FF861D", fontWeight: "bold" }} />
              <Chip label="8 min read" sx={{ backgroundColor: "rgba(255, 255, 255, 0.1)", color: "#aaa" }} />
            </Box>

            <Typography variant="h3" sx={{ fontWeight: "bold", color: "white", marginBottom: "1rem" }}>
              Migrating to Linear 101
            </Typography>
            <Typography variant="body1" sx={{ color: "rgba(255, 255, 255, 0.8)", marginBottom: "0rem" }}>
              Linear helps streamline software projects, sprints, tasks, and bug tracking. Here’s how to get started.
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
              <img
                src={heroImage}
                alt="Hero"
                style={{
                  width: "100%",
                  height: "auto",
                  borderRadius: "12px",
                  // objectFit: "cover",
                }}
              />
            </Box>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
}
