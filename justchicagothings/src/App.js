// src/App.jsx
import React, { useState } from 'react';
import { Routes, Route, useNavigate } from 'react-router-dom';
import Header from './components/Header';
import Hero from './components/Hero';
import FeaturedPicks from './components/FeaturedPicks';
import EventDetailPage from './components/EventDetailPage';
import Footer from './components/Footer';
import BuyCoffeeDialog from './components/BuyCoffeeDialog';
import { redirectToCheckout } from './stripeFunctions';
import Box from '@mui/material/Box';


export default function App() {
  const [dialogOpen, setDialogOpen] = useState(false);
  const navigate = useNavigate();

  const handleCardClick = (id) => navigate(`/events/${id}`);
  const openDialog = () => setDialogOpen(true);
  const closeDialog = () => setDialogOpen(false);
  const handleBuyCoffee = async () => {
    closeDialog();
    await redirectToCheckout();
  };

  return (
    <>
      <Header />
      <Routes>
        <Route
          path="/"
          element={
            <>
              <Hero onBrowse={openDialog} />
              <FeaturedPicks onCardClick={handleCardClick} />
              <Box display="flex" justifyContent="center" mt={4}>
                <Footer.BuyMoreButton onClick={openDialog} />
              </Box>
              <Footer />
              <BuyCoffeeDialog
                open={dialogOpen}
                onClose={closeDialog}
                onBuy={handleBuyCoffee}
              />
            </>
          }
        />
        <Route path="/events/:id" element={<EventDetailPage />} />
      </Routes>
    </>
  );
}
