import React from 'react';
import { Box, Container, Typography, Grid, Paper, Button } from '@mui/material';

const Projects = () => {
  return (
    <Box sx={{ backgroundColor: '#181818', color: 'white', padding: '4rem 0' }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00', marginBottom: '2rem' }}>
          Our Projects
        </Typography>
        
        <Grid container spacing={4} justifyContent="center">
          {/* Project 1 */}
          <Grid item xs={12} sm={6} md={4}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                height: '100%',
                justifyContent: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <img src="/path/to/web-development-image.png" alt="Web Development" style={{ width: '100%', borderRadius: '8px' }} />
              <Typography variant="h6" sx={{ color: '#ff9f00', marginTop: '1rem' }}>
                Web Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                We provide comprehensive solutions for businesses, including web development, design, and digital strategy.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>

          {/* Project 2 */}
          <Grid item xs={12} sm={6} md={4}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                height: '100%',
                justifyContent: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <img src="/path/to/mobile-app-design-image.png" alt="Mobile App Design & Development" style={{ width: '100%', borderRadius: '8px' }} />
              <Typography variant="h6" sx={{ color: '#ff9f00', marginTop: '1rem' }}>
                Mobile App Design & Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                We create user-friendly mobile applications, focusing on innovative designs and robust development.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>

          {/* Project 3 */}
          <Grid item xs={12} sm={6} md={4}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                height: '100%',
                justifyContent: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <img src="/path/to/web3-development-image.png" alt="Web3 Development" style={{ width: '100%', borderRadius: '8px' }} />
              <Typography variant="h6" sx={{ color: '#ff9f00', marginTop: '1rem' }}>
                Web3 Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                We specialize in creating decentralized applications and blockchain technologies for the future of the web.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>
        </Grid>

        {/* View More Button */}
        <Box sx={{ textAlign: 'center', marginTop: '3rem' }}>
          <Button variant="outlined" sx={{ color: 'white', borderColor: '#ff9f00', borderRadius: '20px' }}>
            View More
          </Button>
        </Box>
      </Container>
    </Box>
  );
};

export default Projects;
