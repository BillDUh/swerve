package frc.lib.helper;

public interface IDashboardProvider {
    void putDashboard();

    default void registerDashboard() {
        DashboardHelper.register(this);
    }
}