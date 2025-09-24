import AsyncStorage from "@react-native-async-storage/async-storage";
import * as Location from "expo-location";
import authService from "../../../database/authService";

const USE_AUTO_KEY = "useAuto";

export async function getZipCode() {
  try {
    // 🔑 Read the flag fresh each time
    const raw = await AsyncStorage.getItem(USE_AUTO_KEY);
    const useAuto = raw ? JSON.parse(raw) : true;

    if (useAuto) {
      const isLocationEnabled = await Location.hasServicesEnabledAsync();
      if (!isLocationEnabled) {
        throw new Error("Location services are disabled. Please enable them in your device settings.");
      }

      const { status, canAskAgain } = await Location.requestForegroundPermissionsAsync();

      if (status !== "granted") {
        if (canAskAgain) {
          throw new Error("Location permission is required to share your location. Please allow access when prompted.");
        } else {
          throw new Error("Location permission was denied. Please enable it in your device settings to share your location.");
        }
      }

      const location = await Location.getCurrentPositionAsync({
        accuracy: Location.Accuracy.Balanced,
        timeout: 15000,
        maximumAge: 60000,
      });

      const addresses = await Location.reverseGeocodeAsync({
        latitude: location.coords.latitude,
        longitude: location.coords.longitude,
      });

      if (!addresses || addresses.length === 0) {
        throw new Error("Unable to determine your location. Please try again.");
      }

      const address = addresses[0];
      if (!address.postalCode) {
        throw new Error("Unable to determine your ZIP code from your current location.");
      }

      return address.postalCode;
    } else {
      const zip = await authService.getZip();
      console.log("current zip", zip);
      return zip;
    }
  } catch (error) {
    if (error.message.includes("Location request timed out")) {
      throw new Error("Location request timed out. Please make sure you have a good GPS signal and try again.");
    }

    if (error.message.includes("Network request failed")) {
      throw new Error("Unable to get location due to network issues. Please check your connection and try again.");
    }

    throw error;
  }
}

export async function setUseAuto(value) {
  await AsyncStorage.setItem(USE_AUTO_KEY, JSON.stringify(value));
  console.log("set useAuto:", value);
}

export async function getUseAuto() {
  const raw = await AsyncStorage.getItem(USE_AUTO_KEY);
  return raw ? JSON.parse(raw) : true;
}

export async function getLocationPermissionStatus() {
  const { status } = await Location.getForegroundPermissionsAsync();
  return status;
}

export async function isLocationAvailable() {
  const hasServices = await Location.hasServicesEnabledAsync();
  const { status } = await Location.getForegroundPermissionsAsync();
  return hasServices && status === "granted";
}
