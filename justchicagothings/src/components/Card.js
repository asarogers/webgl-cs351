// src/components/Card.jsx
import React from 'react';
import { Card as MuiCard, CardActionArea, CardMedia, CardContent, Typography, Box } from '@mui/material';
import CalendarTodayIcon from '@mui/icons-material/CalendarToday';

export default function Card({ title, description, date, imgSrc, onClick }) {
  return (
    <MuiCard sx={{ maxWidth: 260, m: 1 }} onClick={onClick}>
      <CardActionArea>
        <CardMedia component="img" height="160" image={imgSrc} alt={title} />
        <CardContent>
          <Typography variant="h6" gutterBottom>{title}</Typography>
          <Typography variant="body2" color="text.secondary" paragraph>
            {description}
          </Typography>
          <Box display="flex" alignItems="center" color="primary.main">
            <CalendarTodayIcon fontSize="small" sx={{ mr: 0.5 }} />
            <Typography variant="body2">{date}</Typography>
          </Box>
        </CardContent>
      </CardActionArea>
    </MuiCard>
  );
}
