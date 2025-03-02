import React, { useState } from 'react';
import { Box, Container, Typography, Grid, Paper, Button } from '@mui/material';
import CustomLabel from "../../../components/CustomLabel";
import penPhone from "../img/PenPhone.svg"
import codePhone from "../img/codePhone.svg"
import phone from "../img/phone.svg"
import laptop from "../img/laptop.svg"


const Services = () => {
  const texts = [
    "We build custom mobile apps for Android, iOS, and cross-platform.",
    "Advanced development in the areas of creating applications for local and remote systems using advanced technologies.",
    "Connecting the physical and digital worlds through seamless IoT solutions.",
    "Creating apps that work seamlessly across multiple platforms with one codebase."
  ];
  const imgs = [penPhone, codePhone, phone, laptop]

  return (
    <Box sx={{ color: 'white', padding: '0rem 0' }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00', marginBottom: '2rem' }}>
          <CustomLabel label="Unlock The Future" />
        </Typography>
        <Typography variant="h3" sx={{ textAlign: 'center', fontWeight: "bold", marginBottom: "20px" }}>
          Our <span style={{
            background: "linear-gradient(to top, #FF861D, #FBDF02)",
            WebkitBackgroundClip: "text",
            backgroundClip: "text",
            color: "transparent",
          }}> Services </span>
        </Typography>

        <Grid container spacing={4} justifyContent="center">
          {["Mobile App Design & Development", "Advanced Development", "IoT Development", "Cross-Platform Development"].map((title, index) => (
            <ServiceCard key={index} title={title} text={texts[index]} imgs={imgs[index]} />
          ))}
        </Grid>
      </Container>
    </Box>
  );
};

const ServiceCard = ({ title, text, imgs }) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <Grid item xs={12} sm={6} md={3}>
      <Paper
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        sx={{
          height: "100%", // Ensures all cards are equal height
          display: "flex",
          flexDirection: "column",
          justifyContent: "center",
          padding: "10px",
          backgroundColor: "rgba(58, 58, 58, 0.17)",
          border: "1px solid rgba(255, 255, 255, 0.3)",
          borderRadius: "12px",
          transition: "all 0.5s ease-in-out",
          "&:hover": {
            cursor: "pointer",
            transform: "scale(1.05)",
            background: "linear-gradient(to top, #FF861D, #FBDF02)",
            boxShadow: "0px 6px 25px rgba(255, 255, 255, 0.15), 0px 0px 12px rgba(255, 190, 70, 0.5) inset",
          },
        }}
      >
        <Paper
          sx={{
            display: "flex",
            flexDirection: "column",
            justifyContent: "space-between",
            padding: "20px",
            borderRadius: "12px",
            textAlign: "center",
            height: "100%",
            position: "relative",
            boxShadow: "0px 4px 20px rgba(255, 255, 255, 0.1), 0px 0px 5px rgba(255, 255, 255, 0.2) inset",
            background: isHovered ? "linear-gradient(to top, #FF861D, #FBDF02)" : "rgba(46, 45, 43, 0.1)",
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
              background: isHovered ? "rgba(29, 24, 8, 0.9)" : "linear-gradient(to top, rgba(254, 248, 181, 0.5), rgba(255, 248, 161, 0.4), rgba(228, 190, 76, 0.2))",
              "-webkit-mask": "linear-gradient(#fff 0 0) content-box, linear-gradient(#fff 0 0)",
              "-webkit-mask-composite": "destination-out",
              maskComposite: "exclude",
            }
          }}
        >
          {/* Icon */}
          <Box
            sx={{
              width: "60px", // Adjust the size of the circle
              height: "60px",
              display: "flex",
              alignItems: "bottom",
              justifyContent: "center",
              borderRadius: "50%",
              background: isHovered ? "black" : "rgba(117, 117, 109, 0.5)", // Black background on hover
              transition: "all 0.5s ease-in-out",
            }}
          >

            <defs>
              <img
                src={imgs}
                alt="App Development Icon"
                style={{
                  width: "50px",
                  marginTop: "0.5rem",
                  filter: isHovered ? "invert(38%) sepia(100%) saturate(100%) hue-rotate(1deg) brightness(100%) contrast(105%)" : ""
                }}
              />
            </defs>
          </Box>
          <Typography variant="h6"
            sx={{
              color: isHovered ? "black" : "white",
              fontWeight: "bold",
              fontSize: "clamp(0.75rem, 1.5vw, 1.1rem)",
              marginBottom: "0.5rem",
            }}
          >
            {title}
          </Typography>

          {/* Description */}
          <Typography variant="body2" sx={{
            color: isHovered ? "black" : "white",
            flexGrow: 1
          }}>
            {text}
          </Typography>

          {/* Button with Smooth Transition */}
          <Button
            variant="call_to_action"
            sx={{
              transition: "all 0.5s ease-in-out",
              background: isHovered ? "rgb(255, 255, 255)" : "linear-gradient(to top, #FF861D, #FBDF02)",
              color: isHovered ? "#000" : "#000",
              boxShadow: isHovered
                ? "0 10px 15px rgba(0, 0, 0, 0.3)"
                : "0px 3px 8px rgba(255, 190, 70, 0.5)",
              transform: isHovered ? "scale(1.05)" : "scale(1)",
              "&:hover": {
                background: 'rgb(0, 0, 0)',
              }
            }}
          >
            Make An Appointment
          </Button>
        </Paper>
      </Paper>
    </Grid>
  );
};

export default Services;
