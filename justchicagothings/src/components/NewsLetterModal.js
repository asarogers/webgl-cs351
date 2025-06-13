// src/components/NewsletterModal.jsx
import React, { useState } from 'react';
import { Button } from '@mui/material';
import { GoogleOAuthProvider } from '@react-oauth/google';

// TODO: replace with your OAuth 2.0 Client ID
const GOOGLE_CLIENT_ID = '74814662062-h99b0mr3mvr2q2jq4r1nnuo496qfh3a3.apps.googleusercontent.com ';

// Build the raw RFC2822 message for Gmail API
function makeRawEmail(to, subject, body) {
  const message = [
    `To: ${to}`,
    'Content-Type: text/plain; charset=UTF-8',
    'MIME-Version: 1.0',
    `Subject: ${subject}`,
    '',
    body,
  ].join('\r\n');

  return window.btoa(unescape(encodeURIComponent(message)))
    .replace(/\+/g, '-')
    .replace(/\//g, '_');
}

export default function NewsletterModal() {
  const [subscribed, setSubscribed] = useState(false);

  const handleSubscribe = () => {
    // Initialize the One-Tap token client for Gmail send
    const tokenClient = window.google.accounts.oauth2.initTokenClient({
      client_id: GOOGLE_CLIENT_ID,
      scope: 'https://www.googleapis.com/auth/gmail.send',
      callback: async (tokenResponse) => {
        const { access_token } = tokenResponse;
        const raw = makeRawEmail(
          'hello@justchicagothing.com',
          'New Newsletter Subscriber',
          'A new subscriber just signed up via your site!'
        );
        try {
          await fetch('https://www.googleapis.com/gmail/v1/users/me/messages/send', {
            method: 'POST',
            headers: {
              Authorization: `Bearer ${access_token}`,
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ raw }),
          });
          setSubscribed(true);
        } catch (err) {
          console.error('Failed to send email:', err);
        }
      },
    });

    // Immediately pop up the Gmail account chooser
    tokenClient.requestAccessToken();
  };

  return (
    <GoogleOAuthProvider clientId={GOOGLE_CLIENT_ID}>
      <Button
        variant="contained"
        color="primary"
        disabled={subscribed}
        onClick={handleSubscribe}
      >
        {subscribed ? 'You’re Subscribed!' : 'Subscribe'}
      </Button>
    </GoogleOAuthProvider>
  );
}
