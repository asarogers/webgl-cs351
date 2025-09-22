import { Tabs } from 'expo-router';
import React from 'react';

import { HapticTab } from '@/components/haptic-tab';
import { IconSymbol } from '@/components/ui/icon-symbol';
import { Colors } from '@/constants/theme';
import { useColorScheme } from '@/hooks/use-color-scheme';

export default function TabLayout() {
  const colorScheme = useColorScheme();

  return (
<Tabs
  screenOptions={{
    headerShown: false,
    tabBarStyle: { display: 'none' }, // hides the tab bar globally
  }}
>
  <Tabs.Screen
    name="index"
    options={{
      title: 'Home',
      tabBarIcon: ({ color }) => (
        <IconSymbol size={28} name="house.fill" color={color} />
      ),
      // Hide only for this screen:
      tabBarStyle: { display: 'none' },
    }}
  />
</Tabs>

  );
}
