gcloud functions deploy euclidean_distance \
--gen2 \
--runtime=python312 \
--region=europe-west3 \
--source=. \
--entry-point=euclidean_distance \
--trigger-http
