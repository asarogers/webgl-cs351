import React from 'react';
import { Box, Container, Typography, Grid, Paper } from '@mui/material';
import { Code } from '@mui/icons-material';

const AboutUs = () => {
  return (
    <Box sx={{ backgroundColor: '#181818', color: 'white', padding: '4rem 0' }}>
      <Container maxWidth="lg">
        <Grid container spacing={4}>
          {/* Left Side: Image / Code Block */}
          <Grid item xs={12} md={6}>
            <Paper sx={{ backgroundColor: '#222', padding: '2rem', borderRadius: '8px' }}>
              <Typography variant="h5" sx={{ fontWeight: 'bold', color: '#ff9f00', marginBottom: '1rem' }}>
                Innovating Mobile App Solutions for Your Success
              </Typography>
              <Typography variant="body1" sx={{ color: 'white' }}>
                At Aethur Tech, We're Dedicated to Delivering Innovative Mobile App Solutions that drive your success. Whether you're launching a new app or improving an existing one, Our Team works closely with You to Understand Your Vision and Business Goals. We leverage the Latest Technologies and Industry Best Practices to create Custom, Tailor-Made Apps that drive User Engagement and Provide a Seamless User Experience. Your Success is Our Priority, and We're Committed to Building Solutions that Help You Achieve It.
              </Typography>
            </Paper>
          </Grid>

          {/* Right Side: Code Elements / Visual */}
          <Grid item xs={12} md={6}>
            <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
              {/* You can place a visual of coding elements and icons similar to your design here */}
              <Code sx={{ fontSize: 80, color: '#ff9f00' }} />
              {/* Add more visuals or images if necessary */}
            </Box>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
};

export default AboutUs;
