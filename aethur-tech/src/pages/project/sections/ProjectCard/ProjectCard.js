import React from "react";
import { Box, Container, Typography, Button, Grid, Paper } from "@mui/material";
import { useParams, useNavigate } from "react-router-dom";
import { Facebook, Twitter, LinkedIn } from "@mui/icons-material";
import project1 from "../../imgs/project1.png";
import project2 from "../../imgs/project2.png";
import project3 from "../../imgs/project3.png";
import article1 from "../../imgs/article1.png";
import article2 from "../../imgs/article2.png";
import article3 from "../../imgs/article3.png";
import HeroSection from "./HeroSection";
import Navbar from "../../../../components/Navbar";

// Sample project data
const projectData = {
  "1": {
    title: "Metahumans: Adapting to Unreal 5.2",
    img: project1,
    date: "March 24, 2024",
    author: "John Doe",
    category: "Development",
    description: `The development of metahumans using Unreal Engine 5.2 has revolutionized 3D character rendering. In this article, we explore how this technology enhances realism and motion capture.`,
    sections: [
      {
        subtitle: "Understanding Metahumans",
        content: "Metahumans provide an unprecedented level of realism, enabling artists to craft highly detailed, lifelike digital characters with minimal effort.",
        img: project2,
      },
      {
        subtitle: "Real-Time Performance Capture",
        content: "The integration of AI-based motion tracking allows real-time performance capture, significantly enhancing efficiency in the animation pipeline.",
        img: project3,
      },
    ],
  },
  "2": {
    title: "Advanced AI in Game Development",
    img: project2,
    date: "April 10, 2024",
    author: "Jane Smith",
    category: "AI & Game Dev",
    description: `Artificial intelligence is shaping the next generation of gaming. This article explores how developers are using AI to create more immersive experiences.`,
    sections: [
      {
        subtitle: "Procedural Content Generation",
        content: "AI is being leveraged to create dynamic game worlds, making experiences feel fresh with each playthrough.",
        img: project1,
      },
      {
        subtitle: "AI-Driven NPCs",
        content: "Non-playable characters (NPCs) are becoming more lifelike, learning from players and adapting behavior dynamically.",
        img: project3,
      },
    ],
  },
  "3": {
    title: "The Rise of Web3 Applications",
    img: project3,
    date: "May 5, 2024",
    author: "Michael Doe",
    category: "Blockchain",
    description: `Web3 applications are transforming digital ownership and security. Learn how blockchain is shaping the future of decentralized applications.`,
    sections: [
      {
        subtitle: "Smart Contracts & Security",
        content: "With smart contracts, users can engage in transactions without intermediaries, ensuring security and transparency.",
        img: project1,
      },
      {
        subtitle: "Decentralized Identity",
        content: "Web3 introduces new ways to authenticate identity, reducing reliance on centralized services.",
        img: project2,
      },
    ],
  },
};

// Sample Related Articles
const relatedArticles = [
  { title: "Advanced 3D Modeling Tips", img: article1 },
  { title: "The Future of AI in Animation", img: article2 },
  { title: "Mastering Unreal Engine 5", img: article3 },
];

const ProjectCard = () => {
  const { id } = useParams();
  const navigate = useNavigate();
  const project = projectData[id];

  if (!project)
    return (
      <Typography variant="h5" sx={{ color: "red", textAlign: "center", marginTop: "2rem" }}>
        Project Not Found
      </Typography>
    );

  return (
    <Box sx={{ backgroundColor: "black", color: "white" }}>
     

      {/* Main Content */}
      <Container maxWidth="lg" sx={{ marginTop: "3rem" }}>
      < Navbar/>
      {/* Hero Section */}
      <HeroSection heroImage = {project.img}/>
        <Grid container spacing={4}>
          {/* Left Content Section */}
          <Grid item xs={12} md={8}>
            <Typography variant="body1" sx={{ marginBottom: "1rem", color: "rgba(255,255,255,0.8)" }}>
              {project.description}
            </Typography>

            {/* Project Sections with Images */}
            {project.sections.map((section, index) => (
              <Box key={index} sx={{ marginBottom: "2rem" }}>
                <Typography variant="h5" sx={{ fontWeight: "bold", marginBottom: "0.5rem" }}>
                  {section.subtitle}
                </Typography>
                <Typography variant="body1" sx={{ marginBottom: "1rem", color: "rgba(255,255,255,0.8)" }}>
                  {section.content}
                </Typography>
                <img
                  src={section.img}
                  alt={section.subtitle}
                  style={{
                    width: "100%",
                    borderRadius: "10px",
                  }}
                />
              </Box>
            ))}

            {/* Back Button */}
            <Button
              onClick={() => navigate("/portfolio")}
              sx={{
                marginTop: "1rem",
                color: "#FF861D",
                fontWeight: "bold",
                textTransform: "none",
                "&:hover": { textDecoration: "underline" },
              }}
            >
              ⬅ Back to Portfolio
            </Button>
          </Grid>

          {/* Right Sidebar Section */}
          <Grid item xs={12} md={4}>
            <Paper sx={{ padding: "1.5rem", backgroundColor: "#111", borderRadius: "10px" }}>
              <Typography variant="h6" sx={{ fontWeight: "bold", marginBottom: "1rem", color: "#FF861D" }}>
                Article Details
              </Typography>
              <Typography variant="body2" sx={{ color: "rgba(255,255,255,0.7)" }}>
                <strong>Author:</strong> {project.author}
              </Typography>
              <Typography variant="body2" sx={{ color: "rgba(255,255,255,0.7)", marginBottom: "1rem" }}>
                <strong>Date:</strong> {project.date}
              </Typography>
              <Typography variant="body2" sx={{ color: "rgba(255,255,255,0.7)", marginBottom: "1rem" }}>
                <strong>Category:</strong> {project.category}
              </Typography>
            </Paper>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
};

export default ProjectCard;
