import React from 'react';
import { Box, Container, Typography, Grid, Paper, Rating } from '@mui/material';

const Testimonials = () => {
  return (
    <Box sx={{ backgroundColor: '#181818', color: 'white', padding: '4rem 0' }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00', marginBottom: '2rem' }}>
          What Our Client Says
        </Typography>
        
        <Grid container spacing={4} justifyContent="center">
          {/* Testimonial 1 */}
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
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                "I used the tips from Eastern Oregon to enhance my driver’s skills. I’ve learned so much and met some fantastic people doing so! The results speak for themselves, and I definitely recommend this place."
              </Typography>
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                Kelly Hammond
              </Typography>
              <Rating name="client-rating" value={5} readOnly sx={{ color: '#ff9f00' }} />
            </Paper>
          </Grid>

          {/* Testimonial 2 */}
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
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                "I had injury issues with my 22 Points sport bike. The guys questioned me about my riding style and what could be transformed my bike suspension into a ride that performs flawlessly under every type and speed I drive."
              </Typography>
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                Jaydon Rosser
              </Typography>
              <Rating name="client-rating" value={5} readOnly sx={{ color: '#ff9f00' }} />
            </Paper>
          </Grid>

          {/* Testimonial 3 */}
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
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                "In the world of power sports, finding a solution to specific needs that really works is not always easy. But thanks to these guys, I’m now driving with absolute confidence."
              </Typography>
              <Typography variant="h6" sx={{ color: '#ff9f00', marginBottom: '1rem' }}>
                Kevin Jeffery
              </Typography>
              <Rating name="client-rating" value={5} readOnly sx={{ color: '#ff9f00' }} />
            </Paper>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
};

export default Testimonials;
