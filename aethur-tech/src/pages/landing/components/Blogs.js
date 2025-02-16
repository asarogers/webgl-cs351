import React from 'react';
import { Box, Container, Typography, Grid, Paper, Button, TextField } from '@mui/material';

const BlogSection = () => {
  return (
    <Box sx={{ backgroundColor: '#181818', color: 'white', padding: '4rem 0' }}>
      <Container maxWidth="lg">
        <Typography variant="h4" sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00', marginBottom: '2rem' }}>
          Blog
        </Typography>
        
        <Typography variant="h5" sx={{ textAlign: 'center', marginBottom: '2rem' }}>
          Discover Our Latest Articles
        </Typography>

        <Grid container spacing={4} justifyContent="center">
          {/* Article 1 */}
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
              <img src="/path/to/article1-image.png" alt="Article 1" style={{ width: '100%', borderRadius: '8px' }} />
              <Typography variant="h6" sx={{ color: '#ff9f00', marginTop: '1rem' }}>
                Aug 20, 2022
              </Typography>
              <Typography variant="h6" sx={{ color: 'white', marginTop: '1rem' }}>
                A Comprehensive Guide to Navigating and Utilizing Advanced Platform Features
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                This guide covers how to effectively navigate and make the most of our platform’s advanced features.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Read More
              </Button>
            </Paper>
          </Grid>

          {/* Article 2 */}
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
              <img src="/path/to/article2-image.png" alt="Article 2" style={{ width: '100%', borderRadius: '8px' }} />
              <Typography variant="h6" sx={{ color: '#ff9f00', marginTop: '1rem' }}>
                Aug 10, 2022
              </Typography>
              <Typography variant="h6" sx={{ color: 'white', marginTop: '1rem' }}>
                Maximizing Your Potential: Strategies for Leveraging Interactive Tools and Features
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                Learn how to leverage our interactive tools to enhance your platform experience.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Read More
              </Button>
            </Paper>
          </Grid>

          {/* Article 3 */}
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
              <img src="/path/to/article3-image.png" alt="Article 3" style={{ width: '100%', borderRadius: '8px' }} />
              <Typography variant="h6" sx={{ color: '#ff9f00', marginTop: '1rem' }}>
                Aug 5, 2022
              </Typography>
              <Typography variant="h6" sx={{ color: 'white', marginTop: '1rem' }}>
                Unlocking the Full Power of Our Platform: Step-by-Step Instructions for Engaging with Key Features
              </Typography>
              <Typography variant="body2" sx={{ color: 'white', marginBottom: '1rem' }}>
                This article provides detailed instructions on how to engage effectively with the platform’s key features.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Read More
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

        {/* Newsletter Subscription Section */}
        <Box sx={{ textAlign: 'center', marginTop: '4rem', padding: '3rem 0', backgroundColor: '#222' }}>
          <Typography variant="h5" sx={{ color: 'white', marginBottom: '2rem' }}>
            Subscribe Our Newsletter
          </Typography>
          <Typography variant="body1" sx={{ color: 'white', marginBottom: '2rem' }}>
            Subscribe to our Newsletter for the latest updates, new features, and industry insights.
          </Typography>

          <Box sx={{ display: 'flex', justifyContent: 'center', gap: 2 }}>
            <TextField
              label="Enter your email"
              variant="outlined"
              sx={{ width: '300px', backgroundColor: 'white' }}
            />
            <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
              Subscribe
            </Button>
          </Box>
        </Box>
      </Container>
    </Box>
  );
};

export default BlogSection;
