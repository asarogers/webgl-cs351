import React from "react";
import { Box, Container, Grid, Typography, Link, IconButton } from "@mui/material";
import { Facebook, Twitter, Instagram, LinkedIn, YouTube } from "@mui/icons-material";
import logo from "../images/Final Logo/rect1.png";
import backgroundImage from "../images/Footer.png";

const Footer = () => {
  return (
    <Box
      sx={{
        position: "relative",
        backgroundColor: "#0a0a0a",
        color: "#fff",
        padding: "3rem 0",
        backgroundImage: `url(${backgroundImage})`,
        backgroundSize: "cover",
        backgroundPosition: "center",
        backgroundRepeat: "no-repeat",
      }}
    >
      <Container maxWidth="lg">
        <Grid container spacing={4} justifyContent="space-between" alignItems="start">
          {/* Left Section: Logo & Description */}
          <Grid item xs={12} md={4}>
            <Box>
              <img
                src={logo}
                alt="Aethur Tech Logo"
                style={{
                  width: "clamp(100px, 15vw, 175px)",
                  marginBottom: "10px",
                }}
              />
            </Box>
            <Typography variant="body2" sx={{ maxWidth: "280px", marginBottom: "1rem", color: "#ccc" }}>
              We create tailored mobile apps that turn your ideas into innovative solutions for growth.
            </Typography>
            {/* Social Icons */}
            <Box display="flex" gap={1}>
              <IconButton sx={{ color: "#ff9f00", background: "white"}}>
                <Facebook />
              </IconButton>
              <IconButton sx={{ color: "#ff9f00", background: "white" }}>
                <Twitter />
              </IconButton>
              <IconButton sx={{ color: "#ff9f00", background: "white" }}>
                <Instagram />
              </IconButton>
              <IconButton sx={{ color: "#ff9f00", background: "white" }}>
                <LinkedIn />
              </IconButton>
              <IconButton sx={{ color: "#ff9f00", background: "white" }}>
                <YouTube />
              </IconButton>
            </Box>
          </Grid>

          {/* Middle Section: Quick Links */}
          <Grid item xs={12} md={4}>
            <Typography variant="body1" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#ff9f00" }}>
              Quick Links
            </Typography>
            <Grid container direction="column" spacing={1}>
              {["Home", "About Us", "Car For Sale", "Financing", "Vehicle Body Type"].map((item) => (
                <Grid item key={item}>
                  <Link href="#" color="inherit" variant="body2" sx={{ color: "#ccc", textDecoration: "none" }}>
                    {item}
                  </Link>
                </Grid>
              ))}
            </Grid>
          </Grid>

          {/* Right Section: Policies */}
          <Grid item xs={12} md={4}>
            <Typography variant="body1" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#ff9f00" }}>
              Privacy Policy
            </Typography>
            <Grid container direction="column" spacing={1}>
              {["Terms Of Service", "Cookies Policy"].map((item) => (
                <Grid item key={item}>
                  <Link href="#" color="inherit" variant="body2" sx={{ color: "#ccc", textDecoration: "none" }}>
                    {item}
                  </Link>
                </Grid>
              ))}
            </Grid>
          </Grid>
        </Grid>

        {/* Bottom Section: Copyright & Legal Links */}
        <Box sx={{ textAlign: "center", marginTop: "3rem" }}>
          <Typography variant="body2" sx={{ color: "#ccc" }}>
            &copy; 2024 Aethur Tech. All rights reserved.
          </Typography>
          <Box display="flex" justifyContent="center" gap={2} mt={1}>
            <Link href="#" color="inherit" variant="body2" sx={{ color: "#ff9f00", textDecoration: "none" }}>
              Privacy Policy
            </Link>
            <Link href="#" color="inherit" variant="body2" sx={{ color: "#ff9f00", textDecoration: "none" }}>
              Terms of Service
            </Link>
          </Box>
        </Box>
      </Container>
    </Box>
  );
};

export default Footer;
