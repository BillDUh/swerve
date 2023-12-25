package frc.lib.helper;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;

public final class DashboardHelper {
    private static final ArrayList<IDashboardProvider> providers = new ArrayList<>();
    private static boolean isRegistrationValid = false;


    public static void register(IDashboardProvider provider) {
        if (isRegistrationValid) {
            providers.add(provider);
        } else {
            DriverStation.reportWarning("Found dashboard registries when DashboardHelper is invalid!", true);
        }
    }

    public static void putAllRegistries() {
        providers.forEach(IDashboardProvider::putDashboard);
    }

    public static void enableRegistration() {
        providers.clear();
        isRegistrationValid = true;
    }

    public static void disableRegistration() {
        isRegistrationValid = false;
    }
}