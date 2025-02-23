import { useEffect, useRef } from "react";
import HeroSection from "./components/HeroSection";
import { AppBar, Toolbar, Typography, Box, Button, Grid, Container } from "@mui/material";
import Navbar from "../../components/Navbar";
import AboutUs from "./components/AboutUs";
import Services from "./components/OurServices";
import Testimonials from "./components/Testimonials";
import Projects from "./components/Projects";
import BlogSection from "./components/Blogs";

// Import the function that sets up the Three.js scene
import Canvas  from "./Canvas";

export default function Landing() {
    return (
      <Box
        sx={{
          position: "relative",
          minHeight: "100vh", // Changed to 100vh
          paddingBottom: "3rem",
          backgroundImage: `linear-gradient(rgba(10,10,10,0.25), rgba(10,10,10,0.8))`,
          backgroundSize: "cover",
          backgroundPosition: "center",
          backgroundRepeat: "no-repeat",
        }}
      >
        <Canvas />
        <Container maxWidth="lg" sx={{ zIndex: 3, position: "relative"}}>
          <Navbar />
          <HeroSection />
          <AboutUs />
          <Services />
          <Testimonials />
          <Projects />
          <BlogSection />
        </Container>
      </Box>
    );
  }