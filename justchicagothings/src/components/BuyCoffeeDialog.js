// src/components/BuyCoffeeDialog.jsx
import React from 'react';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Button,
  Typography,
  IconButton
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';

export default function BuyCoffeeDialog({ open, onClose, onBuy, onContinue }) {
  return (
    <Dialog open={open} onClose={onClose}>
      <DialogTitle>
        Enjoying the vibes?
        <IconButton
          aria-label="close"
          onClick={onClose}
          sx={{ position: 'absolute', right: 8, top: 8 }}
        >
          <CloseIcon />
        </IconButton>
      </DialogTitle>
      <DialogContent dividers>
        <Typography>Would you like to buy me a coffee?</Typography>
      </DialogContent>
      <DialogActions sx={{ justifyContent: 'center', pb: 2, px: 3, pt: 1 }}>
        {/* Very small Continue button */}
        <Button
          size="small"
          variant="text"
          onClick={onContinue || onClose}
          sx={{ mr: 1 }}
        >
          No
        </Button>
        <Button
          size="small"
          variant="contained"
          color="primary"
          onClick={onBuy}
        >
          ☕ Buy me a coffee
        </Button>
      </DialogActions>
    </Dialog>
  );
}
