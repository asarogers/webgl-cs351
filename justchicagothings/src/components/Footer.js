// src/components/Footer.jsx
import React, { useState } from 'react';
import Box        from '@mui/material/Box';
import Container  from '@mui/material/Container';
import Grid       from '@mui/material/Grid';
import Typography from '@mui/material/Typography';
import Link       from '@mui/material/Link';
import Button     from '@mui/material/Button';
import NewsletterModal from './NewsLetterModal';

const LINKS = [
  { id: 'taste-of-randolph', label: 'Taste of Randolph' },
  { id: 'summer-smash',      label: 'Summer Smash Festival' },
  { id: 'wells-art-fest',    label: 'Wells Art Fest' },
];

function Footer() {
  const [subscribed, setSubscribed] = useState(false);

  const handleSubscribe = () => {
    // Trigger the native email chooser
    window.location.href =
      'mailto:hello@justchicagothing.com'
      + '?subject=JustChicagoThings%20Newsletter'
      + '&body=Please%20add%20me%20to%20your%20mailing%20list!';
    // Update state so button shows “You’re Subscribed!”
    setSubscribed(true);
  };

  return (
    <Box component="footer" bgcolor="grey.900" color="grey.100" py={6}>
      <Container maxWidth="lg">
        <Grid container spacing={4}>
          {/* Get in Touch */}
          <Grid item xs={12} md={4}>
            <Typography variant="h6" gutterBottom>
              Get in Touch
            </Typography>
            <Box component="ul" sx={{ listStyle: 'none', p: 0, m: 0 }}>
              <li>✉️ hello@justchicagothing.com</li>
              <li>📞 +1 (264) 857-680</li>
              <li>📷 @justchicagothings</li>
            </Box>
          </Grid>

          {/* Quick Links */}
          <Grid item xs={12} md={4}>
            <Typography variant="h6" gutterBottom>
              Quick Links
            </Typography>
            <Box component="ul" sx={{ listStyle: 'none', p: 0, m: 0 }}>
              {LINKS.map(link => (
                <li key={link.id} style={{ marginBottom: 4 }}>
                  <Link href={`#${link.id}`} color="inherit" underline="hover">
                    {link.label}
                  </Link>
                </li>
              ))}
            </Box>
          </Grid>

          {/* Stay in Touch */}
          <Grid item xs={12} md={4}>
            <Typography variant="h6" gutterBottom>
              Stay in Touch
            </Typography>
            <Typography variant="body2" paragraph>
              No spam — just exclusive goodies and event tips.
            </Typography>
            <NewsletterModal />
          </Grid>
        </Grid>

        {/* Bottom Line */}
        <Box textAlign="center" pt={6}>
          <Typography variant="caption" color="grey.500">
            © 2025 JustChicagoThings.com &bull;{' '}
            <Link href="#" color="inherit" underline="hover">
              Privacy Policy
            </Link>{' '}
            &bull;{' '}
            <Link href="#" color="inherit" underline="hover">
              Terms of Service
            </Link>
          </Typography>
        </Box>
      </Container>
      
    </Box>
  );
}

// “View More” helper button — import Footer and use <Footer.BuyMoreButton />
Footer.BuyMoreButton = function BuyMoreButton(props) {
  return (
    <div style={{ height: '100px' }}>
      <Button variant="outlined" color="inherit" {...props}>
        View More
      </Button>
    </div>
  );
};

export default Footer;
