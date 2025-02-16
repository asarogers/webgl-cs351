import HeroSection from "./components/HeroSection";
import { AppBar, Toolbar, Typography, Box, Button, Grid, Container } from "@mui/material";
import backgroundImage from "./img/background1.png"; // Import background image
import Navbar from "../../components/Navbar";

export default function Landing() {
    return (
        <Box
            sx={{
                position: "relative",
                backgroundColor: "#0a0a0a", // Fallback color
                color: "#fff",
                minHeight: "100vh",
                paddingBottom: "3rem",
                backgroundImage: `linear-gradient(rgba(10,10,10,0.25), rgba(10,10,10,0.8)), url(${backgroundImage})`,
                backgroundSize: "cover",
                backgroundPosition: "center",
                backgroundRepeat: "no-repeat",
            }}
        >
            <Container maxWidth="lg">
                <Navbar />
                <HeroSection />
            </Container>
        </Box>
    )
}