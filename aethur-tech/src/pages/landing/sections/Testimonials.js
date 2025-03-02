import React from 'react';
import { Box, Container, Typography, Grid, Paper, Rating, Avatar } from '@mui/material';
import StarIcon from "@mui/icons-material/Star";

import CustomLabel from "../../../components/CustomLabel";
const Testimonials = () => {
  const texts = [
    "Made the trip from eastern oregon to purchase our cf moto 800 xc and had a great experience nice people and Gary was great and had everything ready to go to make our experience fast and easy would recommend this pla...",
    "I had injury issues with my 22 Points sport bike. The guys questioned me about my riding style and what could be transformed my bike suspension into a ride that performs flawlessly under every type and speed I drive.",
    "In the world of power sports, finding a solution to specific needs that really works is not always easy. But thanks to these guys, I’m now driving with absolute confidence.",

  ];
  const names = ["Kelly Runnels", "Jaydon Rosser", "Kevin Jeffery"]
  return (
    <Box sx={{ color: 'white', paddingTop: '4rem' }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00', marginBottom: '0rem' }}>
          <CustomLabel label="Testimonals" />
        </Typography>
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ffffff', marginBottom: '2rem' }}>
          What Our
          <span style={{
            background: "linear-gradient(to top, #FF861D, #FBDF02)",
            WebkitBackgroundClip: "text",
            backgroundClip: "text",
            color: "transparent",
          }}> Client </span>
          Says
        </Typography>

        <Grid container spacing={4} justifyContent="center">
          {

            texts.map((text, index) => {
              return <ReviewCard text={text} name={names[index]} />
            })
          }
        </Grid>
      </Container>
    </Box>
  );
};

const ReviewCard = ({ text, name }) => {
  return (
    <Grid item xs={12} sm={6} md={4}>
      <Paper
        sx={{
          padding: "5px",
          backgroundColor: "#222",
          borderRadius: "12px",
          display: "flex",
          height: "100%",
          flexDirection: "column",
          justifyContent: "center",
          transition: "transform 0.3s, box-shadow 0.3s",
          "&:hover": {
            transform: "scale(1.03)",
            backgroundColor: "#333",
          },
        }}
      >
        {/* Review Content */}
        <Typography
          variant="body2"
          sx={{
            color: "rgba(255, 255, 255, 0.8)",
            fontSize: "0.9rem",
            lineHeight: "1.5",
            marginLeft: "0.55rem",
            marginBottom: "0.55rem",
          }}
        >
          {text}

          <Typography
            component="span"
            sx={{
              color: "#FFB300",
              fontWeight: "bold",
              cursor: "pointer",
              "&:hover": { textDecoration: "underline" },
            }}
          >
            Read More
          </Typography>
        </Typography>

        {/* User Profile Section */}
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            marginTop: "1rem",
          }}
        >
          <Avatar
            src="/path-to-profile-image.jpg" // Replace with actual image path
            alt="Kelly Runnels"
            sx={{ width: 50, height: 50, marginRight: "0.75rem" }}
          />
          <Box>
            <Typography
              variant="h6"
              sx={{
                color: "white",
                fontWeight: "bold",
                fontSize: "1rem",
              }}
            >
              {name}
            </Typography>
            <Typography
              variant="body2"
              sx={{ color: "rgba(255, 255, 255, 0.7)", fontSize: "0.8rem" }}
            >
              Birmingham, AL
            </Typography>
            {/* Star Rating */}
            <Box sx={{ display: "flex", marginTop: "0.3rem" }}>
              {[...Array(5)].map((_, index) => (
                <StarIcon key={index} sx={{ color: "#FFA900", fontSize: "1rem", marginRight: "2px" }} />
              ))}
            </Box>
          </Box>
        </Box>
      </Paper>
    </Grid>
  );
};


export default Testimonials;
