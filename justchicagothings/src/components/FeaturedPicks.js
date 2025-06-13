// src/components/FeaturedPicks.jsx
import React from 'react';
import { Box, Typography, Grid } from '@mui/material';
import Card from './Card';
import { events } from '../data/events';

export default function FeaturedPicks({ onCardClick }) {
  return (
    <Box id="top-events" textAlign="center" py={4} px={2}>
      <Typography variant="h4" gutterBottom>
        This Summer’s <Box component="span" color="primary.main">Top Events</Box>
      </Typography>
      <Grid container justifyContent="center">
        {events.map(evt => (
          <Grid item key={evt.id}>
            <Card
              title={evt.title}
              description={evt.descriptionShort}
              date={evt.date}
              imgSrc={evt.img}
              onClick={() => onCardClick(evt.id)}
            />
          </Grid>
        ))}
      </Grid>
    </Box>
  );
}
