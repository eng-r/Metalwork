/**
 * UI-only unit conversion layer.
 *
 * Backend/controller/plant math stays in SI (with legacy bar fields already
 * present in the telemetry API). Do not move these conversions into plant math.
 */
export const BAR_TO_PSI = 14.503773773;
export const NEWTON_TO_LBF = 0.2248089439;
export const NM_TO_FTLBF = 0.7375621493;

export const barToPsi = (bar: number): number => bar * BAR_TO_PSI;
export const psiToBar = (psi: number): number => psi / BAR_TO_PSI;

export const newtonToLbf = (newton: number): number =>
  newton * NEWTON_TO_LBF;

export const nmToFtLbf = (nm: number): number => nm * NM_TO_FTLBF;
export const ftLbfToNm = (ftLbf: number): number => ftLbf / NM_TO_FTLBF;

export const mpsToMmPerMin = (mps: number): number => mps * 1000.0 * 60.0;

export const formatDuration = (seconds: number): string => {
  const total = Math.max(0, Math.floor(seconds));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const secs = total % 60;

  if (hours > 0) {
    return `${hours}h ${minutes.toString().padStart(2, '0')}m ${secs
      .toString()
      .padStart(2, '0')}s`;
  }
  return `${minutes}m ${secs.toString().padStart(2, '0')}s`;
};
