// src/stripeFunctions.js
import { loadStripe } from '@stripe/stripe-js';

const PUBLIC_KEY = process.env.REACT_APP_STRIPE_PUBLISHABLE_KEY; // ← must be a pk_ key
const BACKEND   = process.env.REACT_APP_BACKEND_URL;
let stripePromise = null;

function getStripe() {
  if (!stripePromise) {
    stripePromise = loadStripe(PUBLIC_KEY);
  }
  return stripePromise;
}

export async function redirectToCheckout() {
  const stripe = await getStripe();

  const res = await fetch(
    `${BACKEND}/api/create-checkout-session`,
    {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ item: 'coffee', amount: 300 })
    }
  );

  if (!res.ok) {
    window.alert('Failed to create checkout session.');
    return;
  }

  const { sessionId } = await res.json();
  const { error } = await stripe.redirectToCheckout({ sessionId });

  if (error) {
    console.error(error);
    window.alert('Stripe redirect error.');
  }
}
