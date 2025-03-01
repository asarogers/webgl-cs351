import React from 'react';
import { ThemeProvider } from '@mui/material/styles';
import { theme } from './components/colorTheme';
import HeroSection from './pages/landing/sections/HeroSection';
import Footer from './components/Footer';
import "./App.css"
import Landing from './pages/landing/Landing';

function App() {
  return (
    <div className="App" style={{background:"black"}}>
      <ThemeProvider theme={theme}>
        <Landing />
        < Footer/>
      </ThemeProvider>
    </div>
  );
}

export default App;
