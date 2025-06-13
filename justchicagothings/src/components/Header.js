// src/components/Header.jsx
import React from 'react';
import { AppBar, Toolbar, Typography, Link, Box } from '@mui/material';
import { Link as RouterLink } from 'react-router-dom';

export default function Header() {
  return (
    <AppBar position="static" color="transparent" elevation={0}>
      <Toolbar>
        <Typography variant="h6" sx={{ flexGrow: 1, color: 'text.primary' }}>
          Just Chicago Things
        </Typography>
        <Box>
          <Link component={RouterLink} to="/" sx={{ mx: 1 }}>Home</Link>
          <Link href="/#top-events" sx={{ mx: 1 }}>Events</Link>
          <Link component={RouterLink} to="/map" sx={{ mx: 1 }}>Map</Link>
        </Box>
      </Toolbar>
    </AppBar>
  );
}
