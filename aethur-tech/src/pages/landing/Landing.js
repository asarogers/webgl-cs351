import { useEffect, useRef } from "react";
import HeroSection from "./sections/HeroSection";
import { AppBar, Toolbar, Typography, Box, Button, Grid, Container } from "@mui/material";
import Navbar from "../../components/Navbar";
import AboutUs from "./sections/AboutUs";
import Services from "./sections/OurServices";
import Testimonials from "./sections/Testimonials";
import Projects from "./sections/Projects";
import BlogSection from "./sections/Blogs";
import Newsletter from "./sections/Newsletter";

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
          < Newsletter/>
        </Container>
      </Box>
    );
  }