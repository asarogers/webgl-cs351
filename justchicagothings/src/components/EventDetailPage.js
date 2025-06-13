// src/components/EventDetailPage.jsx
import React from 'react';
import { useParams, Link as RouterLink } from 'react-router-dom';
import { Container, Typography, Box, List, ListItem, Button } from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { events } from '../data/events';

export default function EventDetailPage() {
  const { id } = useParams();
  const evt = events.find(e => e.id === id);

  if (!evt) {
    return (
      <Container sx={{ py: 4 }}>
        <Typography variant="h5">Event Not Found</Typography>
        <Button
          component={RouterLink}
          to="/"
          startIcon={<ArrowBackIcon />}
          sx={{ mt: 2 }}
        >
          Back home
        </Button>
      </Container>
    );
  }

  return (
    <Container sx={{ py: 4 }}>
      <Button
        component={RouterLink}
        to="/"
        startIcon={<ArrowBackIcon />}
        sx={{ mb: 2 }}
      >
        Back home
      </Button>

      <Typography variant="h3" gutterBottom>
        {evt.title}
      </Typography>
      <Typography variant="subtitle1" color="primary.main" gutterBottom>
        {evt.date}
      </Typography>

      {/* Constrained and centered image */}
      <Box
        component="img"
        src={evt.img}
        alt={evt.title}
        sx={{
          width: '100%',
          maxWidth: 600,
          display: 'block',
          mx: 'auto',
          borderRadius: 1,
          my: 2
        }}
      />

      {evt.details.length > 1 ? (
        <List>
          {evt.details.map((d, i) => (
            <ListItem key={i} sx={{ display: 'list-item' }}>
              {d}
            </ListItem>
          ))}
        </List>
      ) : (
        <Typography>{evt.details[0]}</Typography>
      )}
    </Container>
  );
}
