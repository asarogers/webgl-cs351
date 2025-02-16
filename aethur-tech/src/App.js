import React from 'react';
import { ThemeProvider } from '@mui/material/styles';
import { theme } from './components/colorTheme';
import HeroSection from './pages/landing/HeroSection';
import "./App.css"

function App() {
  return (
    <div className="App">
      <ThemeProvider theme={theme}>
        <HeroSection />
      </ThemeProvider>
    </div>
  );
}

export default App;
