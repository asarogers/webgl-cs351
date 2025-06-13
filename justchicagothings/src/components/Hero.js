// src/components/Hero.jsx
import React from 'react';
import { Box, Typography, Button } from '@mui/material';

export default function Hero({ onBrowse }) {
  return (
    <Box
      sx={{
        height: '60vh',
        background: `url('/assets/hero.jpg') center/cover no-repeat`,
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        textAlign: 'center', color: '#fff', textShadow: '0 2px 8px rgba(0,0,0,0.6)'
      }}
    >
      <Box maxWidth={600} p={2}>
        <Typography variant="h3" gutterBottom>
          Discover Chicago’s Summer Blockbusters
        </Typography>
        <Typography variant="h6" paragraph>
          All the hottest festivals, food stalls, and art happenings—all in one place.
        </Typography>
        <Button variant="contained" color="primary" onClick={onBrowse}>
          Browse Events
        </Button>
      </Box>
    </Box>
  );
}
