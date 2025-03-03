import React, { useState, useEffect } from "react";
import { Link, useLocation } from "react-router-dom";
import logo from "../images/Final Logo/rect1.png";
import {
  AppBar,
  Toolbar,
  Box,
  Button,
  Typography,
  IconButton,
  Drawer,
  List,
  ListItem,
  ListItemButton,
  ListItemText,
} from "@mui/material";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faBars, faX } from "@fortawesome/free-solid-svg-icons";

export default function Navbar() {
  const [openMenu, setOpenMenu] = useState(false);
  const [isMobile, setIsMobile] = useState(window.innerWidth <= 768);
  const location = useLocation(); // Get current page path

  useEffect(() => {
    const handleResize = () => setIsMobile(window.innerWidth <= 768);
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  }, []);

  const toggleMenu = () => setOpenMenu((prevState) => !prevState);

  // Define links
  const links = ["Home", "About Us", "Blog", "Documentation", "Portfolio"];
  const routes = { Home: "/", Portfolio: "/portfolio" }; // Routes for valid navigation

  return (
    <AppBar position="sticky" sx={{ backgroundColor: "#0a0a0a", color: "#fff", padding: "0", boxShadow: "none" }}>
      <Toolbar
        sx={{
          display: "flex",
          justifyContent: "space-between",
          borderRadius: "10px",
          border: "1px solid rgba(255, 255, 255, 0.33)",
          background: "linear-gradient(to right, rgba(255, 255, 255, 0.05), rgba(238, 237, 237, 0.05))",
          margin: "0px",
        }}
      >
        {/* Logo Section */}
        <Box sx={{ display: "flex", alignItems: "center" }}>
          <img src={logo} alt="Logo" style={{ width: "clamp(100px, 15vw, 175px)", marginBottom: "5px" }} />
        </Box>

        {/* Navigation Links */}
        {!isMobile ? (
          <Box sx={{ display: "flex", justifyContent: "space-evenly", width: "60%" }}>
            {links.map((link) => {
              const isActive = location.pathname === routes[link];
              return routes[link] ? (
                // Valid navigation for Home & Portfolio
                <Link key={link} to={routes[link]} style={{ textDecoration: "none", color: "inherit" }}>
                  <Typography
                    sx={{
                      cursor: "pointer",
                      fontWeight: "bold",
                      position: "relative",
                      backgroundImage: isActive ? "linear-gradient(to top, #FF861D, #FBDF02)" : "none",
                      backgroundClip: isActive ? "text" : "none",
                      WebkitBackgroundClip: isActive ? "text" : "none",
                      color: isActive ? "transparent" : "inherit",
                      transition: "color 0.3s ease, transform 0.3s ease",
                      fontSize: "clamp(0.75rem, 1.5vw, 1.15rem)",
                      "&:hover": { color: "#FFC107", transform: "translateY(-5px)" },
                      "&::after": {
                        content: '""',
                        display: "block",
                        width: isActive ? "100%" : "0%",
                        height: "2px",
                        background: "linear-gradient(to right, #FF861D, #FBDF02)",
                        transition: "width 0.3s ease",
                      },
                    }}
                  >
                    {link}
                  </Typography>
                </Link>
              ) : (
                // Display static links (No navigation)
                <Typography
                  key={link}
                  sx={{
                    cursor: "not-allowed",
                    fontWeight: "bold",
                    fontSize: "clamp(0.75rem, 1.5vw, 1.15rem)",
                    opacity: 0.5, // Dim inactive links
                  }}
                >
                  {link}
                </Typography>
              );
            })}
          </Box>
        ) : (
          <>
            <Button variant="call_to_action" sx={{ marginRight: "60px" }}>
              Contact Us
            </Button>
            <IconButton onClick={toggleMenu} sx={{ color: "#fff" }}>
              <FontAwesomeIcon icon={openMenu ? faX : faBars} />
            </IconButton>
          </>
        )}

        {!isMobile && <Button variant="call_to_action">Contact Us</Button>}
      </Toolbar>

      {/* Mobile Drawer Menu */}
      <Drawer anchor="right" open={openMenu} onClose={toggleMenu}>
        <Box
          sx={{
            width: 250,
            backgroundColor: "#1a1a1a",
            color: "#fff",
            height: "100%",
            display: "flex",
            flexDirection: "column",
            padding: 2,
          }}
        >
          <Typography variant="h6" sx={{ marginBottom: 2, textAlign: "center", fontWeight: "bold" }}>
            Menu
          </Typography>
          <List>
            {links.map((link) => (
              <ListItem key={link} disablePadding>
                {routes[link] ? (
                  // Valid navigation for Home & Portfolio
                  <ListItemButton component={Link} to={routes[link]} onClick={toggleMenu}>
                    <ListItemText
                      primary={link}
                      sx={{
                        color: location.pathname === routes[link] ? "#FFC107" : "#fff",
                        fontWeight: location.pathname === routes[link] ? "bold" : "normal",
                      }}
                    />
                  </ListItemButton>
                ) : (
                  // Static text for other links
                  <ListItemText
                    primary={link}
                    sx={{ opacity: 0.5, paddingLeft: "16px", fontSize: "1rem", color: "#999" }}
                  />
                )}
              </ListItem>
            ))}
          </List>
          <Button
            variant="contained"
            sx={{
              backgroundColor: "#FFC107",
              color: "#000",
              marginTop: "auto",
              fontWeight: "bold",
            }}
          >
            Contact Us
          </Button>
        </Box>
      </Drawer>
    </AppBar>
  );
}
