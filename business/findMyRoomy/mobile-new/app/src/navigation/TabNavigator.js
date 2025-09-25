// TabNavigator.js
import React from "react";
import { createBottomTabNavigator } from "@react-navigation/bottom-tabs";
import { Ionicons } from "@expo/vector-icons";

import MapScreen from "../screens/main/MapScreen";
import DiscoverScreen from "../screens/main/DiscoveryScreen";
// import MessagingScreen from "../screens/main/MessagingScreen";
import AccountScreen from "../screens/main/_AccountScreen";
import ProfileScreen from "../screens/main/ProfileScreen"
import SettingsScreen from "../screens/settings/SettingsHome";

const Tab = createBottomTabNavigator();

const TabNavigator = () => {
  return (
    <Tab.Navigator
      screenOptions={({ route }) => ({
        headerShown: false,
        tabBarStyle: {
          backgroundColor: "#1F2937",
          borderTopColor: "#374151",
        },
        tabBarActiveTintColor: "#3B82F6",
        tabBarInactiveTintColor: "#9CA3AF",
        tabBarIcon: ({ color, size }) => {
          let iconName;

          switch (route.name) {
            case "Map":
              iconName = "map";
              break;
            case "Discover":
              iconName = "search";
              break;
            case "Messages":
              iconName = "chatbubble-ellipses";
              break;
            case "Profile":
              iconName = "person-circle";
              break;
            case "Account":
              iconName = "settings";
              break;
            default:
              iconName = "ellipse";
          }

          return <Ionicons name={iconName} size={size} color={color} />;
        },
      })}
    >
       <Tab.Screen name="Profile" component={ProfileScreen} />
       {/* <Tab.Screen name="Messages" component={MessagingScreen} /> */}
      <Tab.Screen name="Map" component={MapScreen} />
      <Tab.Screen name="Discover" component={DiscoverScreen} />
      <Tab.Screen name="Settings" component={SettingsScreen} />

      

      {/* <Tab.Screen name="Messages" component={MessagingScreen} />
      <Tab.Screen name="Profile" component={ProfilePreviewScreen} />
      <Tab.Screen name="Account" component={EmptyAccountScreen} /> */}
    </Tab.Navigator>
  );
};

export default TabNavigator;
