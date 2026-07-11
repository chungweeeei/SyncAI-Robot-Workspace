import { PageHeader } from "@/components/page-header";
import { AppearanceSettings } from "@/components/settings/appearance-settings";
import { NetworkSettings } from "@/components/settings/network-settings";
import { mockRobotState } from "@/lib/mock/robot-state";

export default function SettingsPage() {
  return (
    <>
      <PageHeader title="Settings" />
      <div className="mx-auto grid w-full max-w-2xl gap-4 p-4">
        <NetworkSettings network={mockRobotState.network_status} />
        <AppearanceSettings />
      </div>
    </>
  );
}
