import React, { useEffect, useState } from "react";
import { createNativeStackNavigator } from "@react-navigation/native-stack";
import AsyncStorage from "@react-native-async-storage/async-storage";
import OnboardingNavigator from "./OnboardingNavigator";
import AuthNavigator from "./AuthNavigator";
import TabNavigator from "./TabNavigator";
import ProfilePreviewScreen from "../screens/main/ProfilePreviewScreen";
import { clearAppCache } from "../../../utils/storage";
import SplashScreen from "../screens/onboarding/SplashScreen";

const Stack = createNativeStackNavigator();

const AppNavigator = () => {
  // Initialize as null to show loading state
  const [isFirstTimeUser, setIsFirstTimeUser] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    const checkFirstTimeUser = async () => {
      // await clearAppCache();
      try {
        const value = await AsyncStorage.getItem("hasLaunched");
        if (value === null) {
          // No value stored, so it's their first time
          setIsFirstTimeUser(true);
        } else {
          // Value exists (regardless of what it is), so not first time
          // setIsFirstTimeUser(false);
          setIsFirstTimeUser(false);

        }
      } catch (err) {
        console.error("Failed to check first-time user flag:", err);
        // On error, assume first time for safety
        setIsFirstTimeUser(true);
      }
    };

    checkFirstTimeUser();
  }, []);

  // This function will persist the onboarding completion
  const completeOnboarding = async () => {
    try {
      console.log("Completing onboarding...");
      await AsyncStorage.setItem("hasLaunched", "true");
      setIsFirstTimeUser(false);
      console.log("Onboarding completed and saved to storage");
    } catch (err) {
      console.error("Failed to save onboarding completion:", err);
    }
  };

  // Show loading/splash while checking storage
  if (isFirstTimeUser === null) {
    return <SplashScreen />; // or return null for blank screen
  }

  return (
    <Stack.Navigator screenOptions={{ headerShown: false }}>
      <Stack.Screen name="Onboarding">
        {props => (
          <OnboardingNavigator
            {...props}
            isFirstTime={isFirstTimeUser}
            completeOnboarding={completeOnboarding}
          />
        )}
      </Stack.Screen>
      <Stack.Screen name="Auth" component={AuthNavigator} />
      <Stack.Screen name="Main" component={TabNavigator} />
      <Stack.Screen name="ProfilePreview" component={ProfilePreviewScreen} />
    </Stack.Navigator>
  );
};

export default AppNavigator;