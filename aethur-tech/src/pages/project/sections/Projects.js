import React, { useState } from "react";
import {
  Box,
  Container,
  Typography,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Button,
} from "@mui/material";
import tamir from "../imgs/TAMIR/tamirVideo.mp4";
import project1 from "../imgs/project1.png";
import project2 from "../imgs/project2.png";
import project3 from "../imgs/project3.png";
import { useNavigate } from "react-router-dom";

const projects = [
  {
    id: 1,
    year: "2024",
    client: "Project 01",
    description: "Comprehensive Mobile App Design and Development",
    media: tamir,
    type: "video",
  },
  {
    id: 2,
    year: "2026",
    client: "Project 02",
    description: "Real-time Chat Application Development",
    media: project2,
    type: "image",
  },
  {
    id: 3,
    year: "2025",
    client: "Project 03",
    description: "Innovative Web3 Commerce Services",
    media: project3,
    type: "image",
  },
];

const Projects = () => {
  const navigate = useNavigate();
  const [hoveredMedia, setHoveredMedia] = useState(null);
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

  const handleProjectClick = (id) => {
    navigate(`/portfolio/projectCard/${id}`);
  };

  const handleMouseEnter = (event, mediaSrc) => {
    setHoveredMedia(mediaSrc);
  };

  const handleMouseMove = (event) => {
    setMousePos({ x: event.clientX + 100, y: event.clientY - 40 });
  };

  const handleMouseLeave = () => {
    setHoveredMedia(null);
  };

  return (
    <Box
      sx={{
        textAlign: "center",
        padding: "0rem 0rem 2rem 0",
        background: "black",
        minHeight: "100%",
        position: "relative",
      }}
    >
      <Container maxWidth="lg">
        <Typography
          variant="h3"
          sx={{ fontWeight: "bold", color: "#fff", marginBottom: "2rem" }}
        >
          Our{" "}
          <span
            style={{
              background: "linear-gradient(to top, #FF861D, #FBDF02)",
              WebkitBackgroundClip: "text",
              backgroundClip: "text",
              color: "transparent",
            }}
          >
            Projects
          </span>
        </Typography>

        {/* Table Container */}
        <TableContainer
          component={Paper}
          sx={{
            background: "transparent",
            borderRadius: "10px",
            overflow: "hidden",
          }}
        >
          <Table>
            <TableHead>
              <TableRow
                sx={{
                  background:
                    "linear-gradient(to top, rgb(87, 70, 56), rgb(44, 43, 38))",
                }}
              >
                <TableCell sx={{ color: "white", fontWeight: "bold" }}></TableCell>
                <TableCell sx={{ color: "white", fontWeight: "bold" }}>Date</TableCell>
                <TableCell sx={{ color: "white", fontWeight: "bold" }}>Client</TableCell>
                <TableCell sx={{ color: "white", fontWeight: "bold" }}>Description</TableCell>
                <TableCell sx={{ color: "white", fontWeight: "bold", textAlign: "center" }}>
                  Action
                </TableCell>
              </TableRow>
            </TableHead>

            {/* Table Body */}
            <TableBody>
              {projects.map((project, index) => (
                <TableRow
                  key={index}
                  sx={{
                    cursor: "pointer",
                    background:
                      index % 2 === 0
                        ? "rgba(255, 255, 255, 0.05)"
                        : "rgba(255, 255, 255, 0.1)",
                    "&:hover": { background: "rgba(255, 255, 255, 0.2)" },
                  }}
                  onMouseEnter={(event) => handleMouseEnter(event, project.media)}
                  onMouseMove={handleMouseMove}
                  onMouseLeave={handleMouseLeave}
                  onClick={() => handleProjectClick(project.id)}
                >
                  <TableCell sx={{ color: "#fff" }}>
                    {project.type === "video" ? (
                      <video
                        src={project.media}
                        width="50"
                        height="50"
                        style={{ borderRadius: "5px" }}
                        muted
                        loop
                      />
                    ) : (
                      <img
                        src={project.media}
                        style={{ width: "50px", borderRadius: "5px" }}
                        alt="Project Preview"
                      />
                    )}
                  </TableCell>
                  <TableCell sx={{ color: "#fff" }}>{project.year}</TableCell>
                  <TableCell sx={{ color: "#fff" }}>{project.client}</TableCell>
                  <TableCell sx={{ color: "#fff" }}>{project.description}</TableCell>
                  <TableCell sx={{ textAlign: "center" }}>
                    <Button
                      variant="contained"
                      sx={{
                        background: "linear-gradient(to right, #FF861D, #FBDF02)",
                        color: "#000",
                        fontWeight: "bold",
                        borderRadius: "20px",
                        padding: "6px 16px",
                        textTransform: "none",
                        "&:hover": {
                          background: "linear-gradient(to right, #FBDF02, #FF861D)",
                        },
                      }}
                    >
                      View
                    </Button>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </Container>

      {/* Hover Preview */}
      {hoveredMedia && (
        <Box
          sx={{
            position: "fixed",
            top: `${mousePos.y}px`,
            left: `${mousePos.x}px`,
            zIndex: 1000,
            pointerEvents: "none",
            transform: "translate(-50%, -50%)",
            transition: "opacity 0.2s ease-in-out",
          }}
        >
          {projects.find((project) => project.media === hoveredMedia)?.type === "video" ? (
            <video
              src={hoveredMedia}
              autoPlay
              loop
              muted
              style={{
                width: "200px",
                height: "auto",
                borderRadius: "10px",
                boxShadow: "0px 4px 10px rgba(255, 255, 255, 0.3)",
              }}
            />
          ) : (
            <img
              src={hoveredMedia}
              alt="Preview"
              style={{
                width: "200px",
                height: "auto",
                borderRadius: "10px",
                boxShadow: "0px 4px 10px rgba(255, 255, 255, 0.3)",
              }}
            />
          )}
        </Box>
      )}
    </Box>
  );
};

export default Projects;
