import React from 'react';
import { Box, Container, Typography, Grid, Paper, Button } from '@mui/material';
import backgroundImage from "../img/projectBackground.png"; // Import background image
import Slider from "react-slick";
import "slick-carousel/slick/slick.css";
import "slick-carousel/slick/slick-theme.css";
import project1 from "../img/project1.png"
import project2 from "../img/project2.png"
import project3 from "../img/project3.png"

const projects = [
  {
    title: "Web Development",
    description: "We provide comprehensive solutions for businesses, including web development, design, and digital strategy.",
    imgSrc: project1,
  },
  {
    title: "Mobile App Design",
    description: "We create user-friendly mobile applications, focusing on innovative designs and robust development.",
    imgSrc: project2,
  },
  {
    title: "Web3 Development",
    description: "We specialize in creating decentralized applications and blockchain technologies for the future of the web.",
    imgSrc: project3,
  },
];

// Carousel settings
const settings = {
  dots: false,
  infinite: true,
  speed: 500,
  slidesToShow: 3, // Show 3 projects at a time
  slidesToScroll: 1,
  autoplay: true,
  autoplaySpeed: 3000,
  arrows: true,
  responsive: [
    {
      breakpoint: 1024, // Medium screens
      settings: {
        slidesToShow: 2,
        slidesToScroll: 1,
      },
    },
    {
      breakpoint: 768, // Small screens
      settings: {
        slidesToShow: 1,
        slidesToScroll: 1,
      },
    },
  ],
};

const Projects = () => {
  return (
    <Box sx={{
      color: 'white', margin: '4rem 0',
      position: "relative",
      backgroundColor: "#0a0a0a",
      minHeight: "100%",
      paddingTop: "4rem",
      paddingBottom: "5px",
      backgroundImage: `linear-gradient(rgba(10,10,10,0.25), rgba(10,10,10,0.8)), url(${backgroundImage})`,
      backgroundSize: "cover",
      backgroundPosition: "center",
      backgroundRepeat: "no-repeat",
      "&::before": {
        content: '""',
        position: "absolute",
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        borderRadius: "150px",
        padding: "1px",
        background: "linear-gradient(to top, rgba(254, 248, 181, 0.5), rgba(255, 248, 161, 0.0), rgba(228, 190, 76, 0.0))",
        "-webkit-mask": "linear-gradient(#fff 0 0) content-box, linear-gradient(#fff 0 0)",
        "-webkit-mask-composite": "destination-out",
        maskComposite: "exclude",
      }
    }}

    // sx={{
    //   display: "flex",
    //   flexDirection: "column",
    //   justifyContent: "space-between",
    //   padding: "20px",
    //   borderRadius: "12px",
    //   textAlign: "center",
    //   height: "100%",
    //   position: "relative",
    //   boxShadow: "0px 4px 20px rgba(255, 255, 255, 0.1), 0px 0px 5px rgba(255, 255, 255, 0.2) inset",
    //   background:  "rgba(46, 45, 43, 0.1)",
    //   transition: "all 0.5s ease-in-out",

    // }}

    >
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#fff', marginBottom: '2rem' }}>
          Our <span style={{
            background: "linear-gradient(to top, #FF861D, #FBDF02)",
            WebkitBackgroundClip: "text",
            backgroundClip: "text",
            color: "transparent",
          }}>Projects</span>
        </Typography>


        <Container maxWidth="lg" height="100%"

        >
          <Slider {...settings}>
            {projects.map((project, index) => (
              <ProjectCard project={project} key={index} />
            ))}
          </Slider>
        </Container>


        {/* View More Button */}
        <Box sx={{ textAlign: 'center', marginTop: '3rem' }}>
          <Button
            variant="outlined"
            sx={{
              position: "relative",
              color: "white",
              borderColor: "#ff9f00",
              borderRadius: "20px",
              overflow: "hidden",
              zIndex: 1, // Add zIndex to ensure text stays above background
              "&::before": {
                content: '""',
                position: "absolute",
                top: 0,
                left: 0,
                width: "100%",
                height: "100%",
                background: "linear-gradient(to top, #FF861D, #FBDF02)",
                opacity: 0,
                transition: "opacity 0.5s ease-in-out",
                zIndex: -1, // Place behind the text
              },
              "&:hover": {
                color: "black", // This wasn't working because the text was behind the gradient
                borderColor: "transparent",
                "&::before": {
                  opacity: 1,
                }
              }
            }}
          >
            View More
          </Button>

        </Box>
      </Container>
    </Box>
  );
};

const ProjectCard = ({ project, index }) => {
  return (
    <Box key={index} sx={{ padding: "0 15px", height: "100%" }}>
      <Paper
        sx={{
          display: "flex",
          flexDirection: "column",
          justifyContent: "space-between",
          padding: "20px",
          marginLeft: "20px",
          borderRadius: "12px",
          textAlign: "center",
          height: "100%", // Ensures uniform height
          position: "relative",
          boxShadow: "0px 4px 20px rgba(255, 255, 255, 0.1), 0px 0px 5px rgba(255, 255, 255, 0.2) inset",
          background: "rgba(46, 45, 43, 0.1)",
          transition: "all 0.5s ease-in-out",
          "&::before": {
            content: '""',
            position: "absolute",
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            borderRadius: "12px",
            padding: "1px",
            background: "linear-gradient(to top, rgba(254, 248, 181, 0.5), rgba(255, 248, 161, 0.4), rgba(228, 190, 76, 0.2))",
            "-webkit-mask": "linear-gradient(#fff 0 0) content-box, linear-gradient(#fff 0 0)",
            "-webkit-mask-composite": "destination-out",
            maskComposite: "exclude",
          }
        }}
      >
        {/* Project Image */}
        <img
          src={project.imgSrc}
          alt={project.title}
          style={{
            width: "100%",
            borderRadius: "8px",
            marginBottom: "1rem",
            objectFit: "cover", // Ensures the image fits well
            maxHeight: "180px", // Limits image height for consistency
          }} />

        {/* Title */}
        <Typography variant="h6" sx={{ color: "rgba(255, 255, 255, 0.8)", marginBottom: "0.5rem" }}>
          {project.title}
        </Typography>

        {/* Description */}
        <Typography variant="body2" sx={{ color: "rgba(167, 167, 167, 0.8)", marginBottom: "1rem" }}>
          {project.description}
        </Typography>

        {/* Call to Action Button */}
        <Typography
          variant="contained"
          sx={{
            color: "#E4BE4C",
            borderRadius: "20px",
            textTransform: "none",

            "&:hover": {
              backgroundColor: "#FFC107",
            },
            textDecoration: "underline"
          }}
        >
          Make an Appointment
        </Typography>
      </Paper>
    </Box>
  )
}

export default Projects;
