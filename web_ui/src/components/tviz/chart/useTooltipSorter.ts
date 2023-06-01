import { useCallback, useState } from "react";
import { Payload } from "recharts/types/component/DefaultTooltipContent";

export const useTooltipSorting = () => {
  const [activeSeries, setActiveSeries] = useState<string | null>(null);

  const onSeriesMouseOver = useCallback((hoveredSeries: string) => {
    setActiveSeries(hoveredSeries);
  }, []);

  const tooltipSorter = useCallback(
    (payload: Payload<any, any>) => {
      if (activeSeries === payload.dataKey) return 0;
      return 1;
    },
    [activeSeries]
  );

  return {
    onSeriesMouseOver,
    tooltipSorter,
  };
};
