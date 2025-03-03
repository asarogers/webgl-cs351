import React from "react";
import {
  Box,
  Container,
  Typography,
  Grid,
  Card,
  CardContent,
  CardMedia,
} from "@mui/material";

import logo from "../../../../images/Final Logo/standAloneLogo.jpg"
const name = "Aethur Tech"

// Sample project data
const projects = [
  {
    id: 1,
    title: "Building your API stack",
    category: "Robotics",
    date: "18 Jan 2024",
    author: name,
    description:
      "The rise of RESTful APIs has been met by a rise in tools for creating, testing, and managing them.",
    img: logo,
  },
  {
    id: 2,
    title: "Building your API stack",
    category: "Robotics",
    date: "18 Jan 2024",
    author: name,
    description:
      "The rise of RESTful APIs has been met by a rise in tools for creating, testing, and managing them.",
    img: logo,
  },
  {
    id: 3,
    title: "Building your API stack",
    category: "Robotics",
    date: "18 Jan 2024",
    author: name,
    description:
      "The rise of RESTful APIs has been met by a rise in tools for creating, testing, and managing them.",
    img: logo,
  },
];

const OtherProjects = () => {
  return (
    <Box sx={{ backgroundColor: "#121212", py: 5 }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ color: "white", mb: 4, textAlign: "center" }}>
          Other Projects
        </Typography>
        <Grid container spacing={3} justifyContent="center">
          {projects.map((project) => (
            <Grid item xs={12} sm={6} md={4} key={project.id}>
              <Card
                sx={{
                  backgroundColor: "#1E1E1E",
                  color: "white",
                  borderRadius: "10px",
                  boxShadow: 3,
                }}
              >
                <CardMedia
                  component="img"
                  height="200"
                  image={project.img}
                  alt={project.title}
                />
                <CardContent>
                  <Typography
                    variant="subtitle2"
                    sx={{ color: "#FDD835", textTransform: "uppercase", fontWeight: 600 }}
                  >
                    {project.category}
                  </Typography>
                  <Typography variant="h6" sx={{ fontWeight: "bold", mt: 1 }}>
                    {project.title}
                  </Typography>
                  <Typography
                    variant="body2"
                    sx={{ color: "rgba(255,255,255,0.7)", mt: 1 }}
                  >
                    {project.description}
                  </Typography>
                  <Box sx={{ display: "flex", alignItems: "center", mt: 2 }}>
                    <img src={logo} style={{width: 40, borderRadius: "25px", marginRight: "5px"}}>
                    </img>
                    <Typography variant="caption" sx={{ color: "gray" }}>
                      {project.author} • {project.date}
                    </Typography>
                  </Box>
                </CardContent>
              </Card>
            </Grid>
          ))}
        </Grid>
      </Container>
    </Box>
  );
};

export default OtherProjects;
