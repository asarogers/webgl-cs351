// WelcomeCarousel.js
import React, { useRef, useState } from "react";
import {
  View,
  FlatList,
  Dimensions,
  StyleSheet,
} from "react-native";
import { FontAwesome5, Entypo } from "@expo/vector-icons";

// Slide components
import FindRoommatesByLocation from "./Location";
import MatchVibe from "./MatchVibe";
import TrustAndReadyScreen from "./TrustAndReadyScreen";
import VerificationLevels from "./VerificationLevels";

const { width } = Dimensions.get("window");

const slides = [
  {
    key: "1",
    title: "Find Roommates by Location",
    subtitle: "Search in your exact neighborhood using our draw-on-map feature.",
    backgroundColor: "#E5F1FF",
    icon: <FontAwesome5 name="map-marked-alt" size={32} color="#3B82F6" />,
  },
  {
    key: "2",
    title: "Match Your Vibe",
    subtitle: "Get personalized roommate matches based on lifestyle and values.",
    backgroundColor: "#D1FADF",
    icon: <Entypo name="star" size={32} color="#22C55E" />,
  },
  {
    key: "3",
    title: "Boost Trust with Verification",
    subtitle: "Get a verified badge and unlock 3x more matches by completing our secure verification steps.",
    backgroundColor: "#1F2937",
    icon: <FontAwesome5 name="shield-alt" size={32} color="#60A5FA" />,
    type: "verification",
  },
  {
    key: "4",
    title: "Ready to Find Your Perfect Roommate?",
    subtitle: "Start exploring matches or create your profile through the app.",
    backgroundColor: "#1E3A8A",
    icon: <FontAwesome5 name="rocket" size={32} color="#FACC15" />,
  },
];

export default function WelcomeCarousel({ navigation, completeOnboarding }){
  const flatListRef = useRef(null);
  const [currentIndex, setCurrentIndex] = useState(0);

  const handleNext = () => {
    if (currentIndex < slides.length - 1) {
      flatListRef.current.scrollToIndex({ index: currentIndex + 1, animated: true });
      // setCurrentIndex(currentIndex + 1); // REMOVE this! Let onMomentumScrollEnd handle it
    } else {
      navigation.replace("UserIntent");
    }
  };
  
  
  const onSkip =()=>{
    navigation.replace("Auth", { screen: "Login" });
  }
  

  const renderItem = ({ item }) => {
    switch (item.key) {
      case "1":
        return <FindRoommatesByLocation handleNext = {handleNext} onSkip={onSkip} />;
      case "2":
        return <MatchVibe handleNext = {handleNext} onSkip={onSkip}/>;
      case "3":
        return <TrustAndReadyScreen handleNext = {handleNext} onSkip={onSkip}/>;
      case "4":
        return <VerificationLevels handleNext = {handleNext} onSkip={onSkip} completeOnboarding={completeOnboarding}/>;
      default:
        return null;
    }
  };

  return (
    <FlatList
      ref={flatListRef}
      data={slides}
      horizontal
      pagingEnabled
      showsHorizontalScrollIndicator={false}
      renderItem={renderItem}
      keyExtractor={(item) => item.key}
      onScroll={(e) => {
        const index = Math.round(e.nativeEvent.contentOffset.x / width);
        setCurrentIndex(index);
      }}
    />
  );
};

const styles = StyleSheet.create({
  slide: {
    width,
    paddingHorizontal: 24,
    alignItems: "center",
    justifyContent: "center",
    flex: 1,
  },
});
