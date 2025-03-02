import React, { useState, useEffect } from "react";
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
  const [activeLink, setActiveLink] = useState("Home");

  useEffect(() => {
    const handleResize = () => setIsMobile(window.innerWidth <= 768);
    window.addEventListener("resize", handleResize);

    return () => window.removeEventListener("resize", handleResize);
  }, []);

  const toggleMenu = () => setOpenMenu((prevState) => !prevState);

  const handleLinkClick = (link) => {
    setActiveLink(link);
    if (isMobile) toggleMenu(); // Close menu on mobile
  };

  return (
    <AppBar
      position="sticky"
      // position="fixed"
      // position="static"
      sx={{
        backgroundColor: "#0a0a0a", 
        color: "#fff",
        padding: "0",
        boxShadow: "none",
      }}
    >
      <Toolbar sx={{
        display: "flex", justifyContent: "space-between", borderRadius: "10px",
        border: "1px solid rgba(255, 255, 255, 0.33)",
        background: "linear-gradient(to right, rgba(255, 255, 255, 0.05), rgba(238, 237, 237, 0.05))",
        margin: "0px 0px 0px 0px"
      }}>
        {/* Logo Section */}
        <Box sx={{ display: "flex", alignItems: "center" }}>
          <img src={logo} alt="Logo" style={{
            width: "clamp(100px, 15vw, 175px)",
            marginBottom: "5px"
          }} />
        </Box>
        {/* Navigation Links */}
        {!isMobile ? (
          <Box sx={{ display: "flex", justifyContent: "space-evenly", width: "60%" }}>
            {["Home", "About Us", "Blog", "Documentation", "Projects"].map((link) => (
              <Typography
                key={link}
                onClick={() => handleLinkClick(link)}
                sx={{
                  cursor: "pointer",
                  fontWeight: "bold",
                  position: "relative",
                  // Apply gradient text color only when the link is active
                  backgroundImage: activeLink === link ? "linear-gradient(to top, #FF861D, #FBDF02)" : "none",
                  backgroundClip: activeLink === link ? "text" : "none",
                  WebkitBackgroundClip: activeLink === link ? "text" : "none",
                  color: activeLink === link ? "transparent" : "inherit",
                  transition: "color 0.3s ease, transform 0.3s ease",
                  fontSize: "clamp(0.75rem, 1.5vw, 1.15rem)",
                  "&:hover": { color: "#FFC107", transform: "translateY(-5px)" },
                  "&::after": {
                    content: '""',
                    display: "block",
                    width: activeLink === link ? "100%" : "0%",
                    height: "2px",
                    background: "linear-gradient(to right, #FF861D, #FBDF02)",
                    transition: "width 0.3s ease",
                  },
                }}
              >
                {link}
              </Typography>
            ))}


          </Box>

        ) : (
          <>
            <Button variant="call_to_action" sx={{marginRight: "60px"}}>Contact Us</Button>

            <IconButton onClick={toggleMenu} sx={{ color: "#fff" }}>
              <FontAwesomeIcon icon={openMenu ? faX : faBars} />
            </IconButton>
          </>
        )}
        {
          !isMobile &&
          <Button variant="call_to_action">Contact Us</Button>
        }
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
          <Typography
            variant="h6"
            sx={{ marginBottom: 2, textAlign: "center", fontWeight: "bold" }}
          >
            Menu
          </Typography>
          <List>
            {["Home", "About Us", "Blog", "Documentation", "Projects"].map((link) => (
              <ListItem key={link} disablePadding>
                <ListItemButton
                  onClick={() => handleLinkClick(link)}
                  sx={{
                    color: activeLink === link ? "#FFC107" : "#fff",
                    "&:hover": { backgroundColor: "#333" },
                  }}
                >
                  <ListItemText primary={link} />
                </ListItemButton>
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
