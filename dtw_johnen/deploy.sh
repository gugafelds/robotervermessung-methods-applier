gcloud functions deploy dtw_johnen \
--gen2 \
--runtime=python312 \
--region=europe-west3 \
--source=. \
--entry-point=dtw_johnen \
--trigger-http
