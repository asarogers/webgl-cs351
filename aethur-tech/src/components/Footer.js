import React from 'react';
import { Box, Container, Grid, Typography, Link, TextField, Button, IconButton } from '@mui/material';
import { Facebook, Twitter, Instagram, LinkedIn, YouTube } from '@mui/icons-material';

const Footer = () => {
  return (
    <Box sx={{ backgroundColor: '#181818', color: 'white', padding: '2rem 0' }}>
      <Container maxWidth="lg">
        <Grid container spacing={4} justifyContent="space-between">
          {/* Left Section */}
          <Grid item xs={12} md={4}>
            <Typography variant="h6" sx={{ fontWeight: 'bold', marginBottom: '1rem' }}>
              Aethur Tech
            </Typography>
            <Typography variant="body2" sx={{ marginBottom: '1rem' }}>
              We create advanced mobile apps to turn your ideas into innovative solutions for growth.
            </Typography>
            <Grid container spacing={2}>
              <Grid item>
                <IconButton sx={{ color: 'white' }}><Facebook /></IconButton>
              </Grid>
              <Grid item>
                <IconButton sx={{ color: 'white' }}><Twitter /></IconButton>
              </Grid>
              <Grid item>
                <IconButton sx={{ color: 'white' }}><Instagram /></IconButton>
              </Grid>
              <Grid item>
                <IconButton sx={{ color: 'white' }}><LinkedIn /></IconButton>
              </Grid>
              <Grid item>
                <IconButton sx={{ color: 'white' }}><YouTube /></IconButton>
              </Grid>
            </Grid>
          </Grid>

          {/* Middle Section */}
          <Grid item xs={12} md={4}>
            <Typography variant="body1" sx={{ fontWeight: 'bold', marginBottom: '1rem' }}>
              Quick Links
            </Typography>
            <Grid container direction="column" spacing={1}>
              <Grid item>
                <Link href="#" color="inherit" variant="body2">Home</Link>
              </Grid>
              <Grid item>
                <Link href="#" color="inherit" variant="body2">About Us</Link>
              </Grid>
              <Grid item>
                <Link href="#" color="inherit" variant="body2">Car For Sale</Link>
              </Grid>
              <Grid item>
                <Link href="#" color="inherit" variant="body2">Financing</Link>
              </Grid>
              <Grid item>
                <Link href="#" color="inherit" variant="body2">Vehicle Body Types</Link>
              </Grid>
            </Grid>
          </Grid>

          {/* Right Section */}
          <Grid item xs={12} md={4}>
            <Typography variant="body1" sx={{ fontWeight: 'bold', marginBottom: '1rem' }}>
              Subscribe Our Newsletter
            </Typography>
            <Typography variant="body2" sx={{ marginBottom: '1rem' }}>
              Subscribe to get the latest mobile development news and insights delivered right to your inbox.
            </Typography>
            <Grid container spacing={1}>
              <Grid item xs={9}>
                <TextField
                  fullWidth
                  variant="outlined"
                  placeholder="Enter your email"
                  sx={{ backgroundColor: 'white' }}
                />
              </Grid>
              <Grid item xs={3}>
                <Button
                  variant="contained"
                  color="primary"
                  fullWidth
                  sx={{ height: '100%' }}
                >
                  Subscribe
                </Button>
              </Grid>
            </Grid>
          </Grid>
        </Grid>

        {/* Bottom Section */}
        <Box sx={{ textAlign: 'center', marginTop: '2rem' }}>
          <Typography variant="body2" color="inherit">
            &copy; 2024 Aethur Tech. All rights reserved.
          </Typography>
          <Grid container spacing={2} justifyContent="center">
            <Grid item>
              <Link href="#" color="inherit" variant="body2">Privacy Policy</Link>
            </Grid>
            <Grid item>
              <Link href="#" color="inherit" variant="body2">Terms of Service</Link>
            </Grid>
            <Grid item>
              <Link href="#" color="inherit" variant="body2">Cookie Policy</Link>
            </Grid>
          </Grid>
        </Box>
      </Container>
    </Box>
  );
};

export default Footer;
