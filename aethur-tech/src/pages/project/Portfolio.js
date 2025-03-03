
import { Container } from "@mui/material"
import Navbar from "../../components/Navbar"
import HeroSection from "./sections/HeroSection"
import Newsletter from "../landing/sections/Newsletter"
import Footer from "../../components/Footer"
import Projects from "./sections/Projects"

export default function Portfolio(){
    return(
        <Container
        sx={{
            position: "relative",
            minHeight: "100vh", // Changed to 100vh
            paddingBottom: "3rem",
            backgroundImage: `linear-gradient(rgba(10,10,10,0.25), rgba(10,10,10,0.8))`,
            backgroundSize: "cover",
            backgroundPosition: "center",
            backgroundRepeat: "no-repeat",
          }}
          >
            < Navbar/>
            <HeroSection />
            <Projects />
            <Newsletter/>

        </Container>
    )
}