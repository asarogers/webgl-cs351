import React from 'react';
import { Box, Container, Typography, Grid, Paper } from '@mui/material';
import { Code } from '@mui/icons-material';
import aboutUsImage from "../img/aboutUs.png"
import CustomLabel from "../../../components/CustomLabel"
const AboutUs = () => {
  return (
    <Box sx={{ color: 'white', padding: '4rem 0' }}>
      <Container maxWidth="lg">
        <Grid container spacing={4}>
          {/* Left Side: Image / Code Block */}
          <Grid item xs={12} md={6} >
            <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
              {/* You can place a visual of coding elements and icons similar to your design here */}
              <img src={aboutUsImage} alt="About Us" />

            </Box>
          </Grid>
          {/* Right Side: Code Elements / Visual */}


          <Grid item xs={12} md={6} >
            <CustomLabel label="About Us" />

            <Typography
              variant="h3"
              sx={{
                fontWeight: "bold",
                margin: "1rem 0",
                lineHeight: "1.2",
                fontSize: "clamp(1.5rem, 4vw, 2.5rem)",
                
              }}
            >
              Innovating
              <span style={{
                background: "linear-gradient(to top, #FF861D,#FBDF02, #FBDF02)", // Gradient background
                WebkitBackgroundClip: "text", // For Safari
                backgroundClip: "text", // Standard
                color: "transparent", // Make text color transparent to show gradient
              }}> Mobile App</span> Solutions for Your Success
            </Typography>
            <Typography variant="body1" sx={{ color: 'white' }}>
              At Aethur Tech, We're Dedicated to Delivering Innovative Mobile App Solutions that drive your success. Whether you're launching a new app or improving an existing one, Our Team works closely with You to Understand Your Vision and Business Goals. We leverage the Latest Technologies and Industry Best Practices to create Custom, Tailor-Made Apps that drive User Engagement and Provide a Seamless User Experience. Your Success is Our Priority, and We're Committed to Building Solutions that Help You Achieve It.
            </Typography>

          </Grid>
        </Grid>
      </Container>
    </Box>
  );
};

export default AboutUs;
