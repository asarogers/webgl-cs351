import React, { useEffect, useState } from "react";
import { Box, Typography, Divider } from "@mui/material";

export default function BlogSidebar({ sections }: { sections: string[] }) {
  const [activeSection, setActiveSection] = useState("");

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        // Find all currently intersecting entries
        const intersecting = entries.filter(entry => entry.isIntersecting);
        
        if (intersecting.length > 0) {
          // Sort by their position from top of viewport
          intersecting.sort((a, b) => {
            return a.boundingClientRect.top - b.boundingClientRect.top;
          });
          
          // Set the topmost intersecting section as active
          const topSection = intersecting[0];
          setActiveSection(topSection.target.id);
        }
      },
      { 
        threshold: 0.1, // Trigger when 10% of the element is visible
        rootMargin: "-100px 0px -50% 0px" // Adjust detection zone
      }
    );

    // Also add a scroll listener as backup for more precise tracking
    const handleScroll = () => {
      const scrollPosition = window.scrollY + 150; // Offset for better UX
      
      for (let i = sections.length - 1; i >= 0; i--) {
        const id = sections[i].replace(/\s+/g, "-").toLowerCase();
        const element = document.getElementById(id);
        
        if (element && element.offsetTop <= scrollPosition) {
          setActiveSection(id);
          break;
        }
      }
    };

    sections.forEach((section) => {
      const id = section.replace(/\s+/g, "-").toLowerCase();
      const el = document.getElementById(id);
      if (el) observer.observe(el);
    });

    // Add scroll listener
    window.addEventListener('scroll', handleScroll, { passive: true });
    
    // Initial check
    handleScroll();

    return () => {
      observer.disconnect();
      window.removeEventListener('scroll', handleScroll);
    };
  }, [sections]);

  const scrollToSection = (id: string) => {
    const el = document.getElementById(id);
    if (el) {
      const y = el.getBoundingClientRect().top + window.scrollY - 100;
      window.scrollTo({ top: y, behavior: "smooth" });
    }
  };

  return (
    <Box
      sx={{
        position: "sticky",
        top: "32px",
        backgroundColor: "white",
        border: "1px solid #e0e0e0",
        borderRadius: "12px",
        boxShadow: "0 2px 8px rgba(0,0,0,0.1)",
        p: 3,
        width: "100%",
        maxWidth: "280px",
        maxHeight: "calc(100vh - 64px)",
        overflowY: "auto",
      }}
    >
      <Typography 
        variant="h6" 
        fontWeight="bold" 
        mb={2}
        sx={{ color: "#1976d2" }}
      >
        📋 Table of Contents
      </Typography>
      
      <Box sx={{ mb: 2 }}>
        {sections.map((title, i) => {
          const id = title.replace(/\s+/g, "-").toLowerCase();
          const isActive = activeSection === id;
          
          return (
            <Typography
              key={i}
              onClick={() => scrollToSection(id)}
              sx={{
                cursor: "pointer",
                mb: 1.5,
                py: 0.5,
                px: 1,
                borderRadius: "6px",
                fontSize: "0.875rem",
                fontWeight: isActive ? "600" : "400",
                color: isActive ? "#1976d2" : "#555",
                backgroundColor: isActive ? "#e3f2fd" : "transparent",
                transition: "all 0.2s ease",
                "&:hover": { 
                  color: "#1976d2",
                  backgroundColor: "#f5f5f5"
                },
                borderLeft: isActive ? "3px solid #1976d2" : "3px solid transparent",
                marginLeft: "-4px",
                paddingLeft: "8px"
              }}
            >
              {title}
            </Typography>
          );
        })}
      </Box>
      
      <Divider sx={{ my: 2 }} />
      
      <Typography 
        variant="caption" 
        sx={{ 
          color: "#777",
          fontStyle: "italic",
          display: "block",
          textAlign: "center"
        }}
      >
        💬 More articles coming soon
      </Typography>
    </Box>
  );
}