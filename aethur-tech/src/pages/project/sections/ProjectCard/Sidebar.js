import React from "react";
import { Box, Typography, TextField, Button, Divider, Grid, IconButton } from "@mui/material";
import { Facebook, Twitter, YouTube, Instagram } from "@mui/icons-material";

export default function Sidebar() {
  return (
    <Box
      sx={{
        width: { xs: "100%", md: "240px" }, // Full width on small screens, fixed on larger screens
        position: { md: "sticky" }, // Stays in place when scrolling
        top: "100px", // Position from the top
        backgroundColor: "#111",
        padding: "1.5rem",
        borderRadius: "12px",
        color: "white",          color:"white",
      }}
    >
      {/* Table of Contents */}
      <Typography variant="h6" sx={{ fontWeight: "bold", color: "#FFCA28", marginBottom: "1rem" }}>
        Introduction
      </Typography>
      <Typography variant="body2" sx={{ marginBottom: "0.5rem", cursor: "pointer", "&:hover": { color: "#FFCA28" } }}>
        Software and tools
      </Typography>
      <Typography variant="body2" sx={{ marginBottom: "0.5rem", cursor: "pointer", "&:hover": { color: "#FFCA28" } }}>
        Other resources
      </Typography>
      <Typography variant="body2" sx={{ marginBottom: "1rem", cursor: "pointer", "&:hover": { color: "#FFCA28" } }}>
        Conclusion
      </Typography>

      <Divider sx={{ backgroundColor: "rgba(255, 255, 255, 0.2)", marginBottom: "1rem" }} />

      {/* Newsletter Section */}
      <Typography variant="body2" sx={{ fontWeight: "bold", color: "#FFCA28", marginBottom: "0.5rem" }}>
        Subscribe to our newsletter
      </Typography>
      <TextField
        placeholder="Enter your email"
        variant="outlined"
        fullWidth
        sx={{
          marginBottom: "0.8rem",
          backgroundColor: "#222",
          borderRadius: "6px",
          "& .MuiOutlinedInput-root": {
            "& fieldset": { borderColor: "rgba(255, 255, 255, 0.3)" },
            "&:hover fieldset": { borderColor: "#FFCA28" },
            "& input": { color: "white" }
          },
        }}
      />
      <Button
        variant="contained"
        fullWidth
        sx={{
          background: "#FFCA28",
          color: "black",
          fontWeight: "bold",
          borderRadius: "8px",
          padding: "10px",
          textTransform: "none",
          "&:hover": { background: "#E6B800" },
        }}
      >
        Join Us
      </Button>

      <Divider sx={{ backgroundColor: "rgba(255, 255, 255, 0.2)", margin: "1rem 0" }} />

      {/* Social Media Icons */}
      <Grid container justifyContent="center" spacing={1}>
        {[Facebook, Twitter, YouTube, Instagram].map((Icon, index) => (
          <Grid item key={index}>
            <IconButton
              sx={{
                backgroundColor: "#222",
                color: "#fff",
                "&:hover": { backgroundColor: "#FFCA28", color: "#000" },
              }}
            >
              <Icon />
            </IconButton>
          </Grid>
        ))}
      </Grid>
    </Box>
  );
}
