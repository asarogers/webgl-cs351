import React from "react";
import { Box, Typography, Button, TextField, Container } from "@mui/material";

export default function Newsletter() {
  return (
    <Container sx={{width: "100%", justifyContent: "center", display: "flex"}}>
      <Box
        sx={{
          marginTop: "4rem",
          padding: "1rem",
          background: "linear-gradient(to top, #FF861D, #FBDF02)", // Updated gradient
          borderRadius: "15px",
          boxShadow: "0px 4px 20px rgba(255, 190, 70, 0.3)",
          width: "80%",
          
        }}
      >
        {/* FLEX CONTAINER FOR LEFT & RIGHT SECTIONS */}
        <Box
          sx={{
            display: "flex",
            justifyContent: "space-between", // Pushes left & right apart
            alignItems: "center",
            width: "100%",
            flexWrap: "wrap", // Ensures responsiveness
            gap: 1,
          }}
        >
          {/* Left Section (Text) */}
          <Box sx={{ width: { xs: "100%", md: "50%" }, textAlign: "left" }}>
            <Typography variant="h5" sx={{ color: "#000", fontWeight: "bold", marginBottom: "0.5rem" }}>
              Subscribe Our Newsletter
            </Typography>

            {/* Description */}
            <Typography variant="body1" sx={{ color: "#000", maxWidth: "500px" }}>
              Subscribe to our newsletter for the latest mobile development and insights delivered to your inbox!
            </Typography>
          </Box>

          {/* Right Section (Input Field & Button) */}
          <Box
  sx={{
    width: { xs: "100%", md: "45%" },
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between", // Pushes TextField left and Button right
    backgroundColor: "rgba(219, 219, 219, 0.65)",
    padding: "0px",
    borderRadius: "30px",
    boxShadow: "0px 4px 10px rgba(0, 0, 0, 0.1)", // Soft shadow
  }}
>
  <TextField
    placeholder="Email Address..."
    variant="standard"
    InputProps={{
      disableUnderline: true,
      sx: {
        padding: "10px 15px",
        fontSize: "1rem",
        borderRadius: "30px",
        flexGrow: 1, // Allows text field to expand
      },
    }}
    sx={{
      backgroundColor: "transparent",
      borderRadius: "30px",
    }}
  />

  {/* Subscribe Button Aligned Right */}
  <Button
    variant="contained"
    sx={{
      backgroundColor: "#fff",
      color: "#000",
      fontWeight: "bold",
      borderRadius: "30px",
      padding: "12px 30px",
      textTransform: "none",
      transition: "all 0.3s ease-in-out",
      "&:hover": {
        backgroundColor: "#f1f1f1",
        boxShadow: "0px 4px 12px rgba(0, 0, 0, 0.2)",
      },
    }}
  >
    Subscribe
  </Button>
</Box>

        </Box>
      </Box>
    </Container>
  );
}
