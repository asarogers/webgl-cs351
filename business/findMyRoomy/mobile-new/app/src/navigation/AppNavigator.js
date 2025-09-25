import React, { useEffect, useState } from "react";
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from "@react-navigation/native-stack";
import AsyncStorage from "@react-native-async-storage/async-storage";
import OnboardingNavigator from "./OnboardingNavigator";
import AuthNavigator from "./AuthNavigator";
import TabNavigator from "./TabNavigator";
import ProfilePreviewScreen from "../screens/main/ProfilePreviewScreen";
import SplashScreen from "../screens/onboarding/SplashScreen";

const Stack = createNativeStackNavigator();

const AppNavigator = ({ isAuthenticated, userOnboarding }) => {
  const [isFirstTimeUser, setIsFirstTimeUser] = useState(null);

  useEffect(() => {
    const checkFirstTimeUser = async () => {
      try {
        const value = await AsyncStorage.getItem("hasLaunched");
        setIsFirstTimeUser(value === null);
      } catch (err) {
        console.error("Failed to check first-time user flag:", err);
        setIsFirstTimeUser(true);
      }
    };

    checkFirstTimeUser();
  }, []);

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

  // Show loading while checking storage
  if (isFirstTimeUser === null) {
    return (

        <SplashScreen />

    );
  }

  // Determine initial route and available screens based on auth state
  let initialRouteName = "Onboarding";
  let availableScreens = [];

  if (isFirstTimeUser) {
    // First time user - show onboarding
    initialRouteName = "Onboarding";
    availableScreens = [
      <Stack.Screen key="onboarding" name="Onboarding">
        {props => (
          <OnboardingNavigator
            {...props}
            isFirstTime={isFirstTimeUser}
            completeOnboarding={completeOnboarding}
          />
        )}
      </Stack.Screen>,
      <Stack.Screen key="auth" name="Auth" component={AuthNavigator} />
    ];
  } else if (isAuthenticated) {
    // Returning authenticated user - go to main app
    initialRouteName = "Main";
    availableScreens = [
      <Stack.Screen key="main" name="Main" component={TabNavigator} />,
      <Stack.Screen key="profilePreview" name="ProfilePreview" component={ProfilePreviewScreen} />
    ];
  } else {
    // Returning user but not authenticated - go to auth
    initialRouteName = "Auth";
    availableScreens = [
      <Stack.Screen key="auth" name="Auth" component={AuthNavigator} />,
      <Stack.Screen key="main" name="Main" component={TabNavigator} />,
      <Stack.Screen key="profilePreview" name="ProfilePreview" component={ProfilePreviewScreen} />
    ];
  }

  return (

      <Stack.Navigator 
        initialRouteName={initialRouteName}
        screenOptions={{ headerShown: false }}
      >
        {availableScreens}
      </Stack.Navigator>

  );
};

export default AppNavigator;