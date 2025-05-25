import numpy as np
def compute_iaecte(cross_track_errors):
 """
 Beregner IAECTE: Integral of Absolute Error – Cross-Track Error.
 Dette representerer total akkumulert lateral avstand mellom fartøyets posisjon
 og nærmeste punkt på referansebanen i hver tidssteg.
 :param vessel_positions: np.array av form (N, 2) – posisjoner [x, y] over tid
 :param reference_trajectory: np.array av form (M, 2) – referansebane [x_ref, y_ref]
 :return: dict med IAECTE, maksimal og gjennomsnittlig cross-track error
 """

 cross_track_errors = np.array(cross_track_errors)

 return {
 "IAECTE": np.sum(cross_track_errors), # Integral of Absolute Error
 "max_CTE": np.max(cross_track_errors), # Maksimalt avvik
 "mean_CTE": np.mean(cross_track_errors) # Gjennomsnittlig avvik
 }