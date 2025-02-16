import React from 'react';
import { Box, Container, Typography, Grid, Paper, Button } from '@mui/material';

const Services = () => {
  return (
    <Box sx={{ backgroundColor: '#181818', color: 'white', padding: '4rem 0' }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00', marginBottom: '2rem' }}>
          Unlock The Future
        </Typography>
        <Typography variant="h5" sx={{ textAlign: 'center', marginBottom: '4rem' }}>
          Our Services
        </Typography>
        
        <Grid container spacing={4} justifyContent="center">
          {/* Service 1 */}
          <Grid item xs={12} sm={6} md={3}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                Mobile App Design & Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                We build custom mobile apps that make space for innovation across industries.
              </Typography>
              <Button variant="contained" sx={{ backgroundColor: '#ff9f00' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>

          {/* Service 2 */}
          <Grid item xs={12} sm={6} md={3}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                Advanced Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                Advanced development in the areas of creating applications for local and remote systems using advanced technologies.
              </Typography>
              <Button variant="contained" sx={{ backgroundColor: '#ff9f00' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>

          {/* Service 3 */}
          <Grid item xs={12} sm={6} md={3}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                IoT Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                Connecting the physical and digital worlds through seamless IoT solutions.
              </Typography>
              <Button variant="contained" sx={{ backgroundColor: '#ff9f00' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>

          {/* Service 4 */}
          <Grid item xs={12} sm={6} md={3}>
            <Paper
              sx={{
                padding: '2rem',
                backgroundColor: '#222',
                borderRadius: '8px',
                textAlign: 'center',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.05)',
                  backgroundColor: '#333',
                },
              }}
            >
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                Cross-Platform Development
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                Creating apps that work seamlessly across multiple platforms with one codebase.
              </Typography>
              <Button variant="contained" sx={{ backgroundColor: '#ff9f00' }}>
                Make an Appointment
              </Button>
            </Paper>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
};

export default Services;
