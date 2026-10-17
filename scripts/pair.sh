#!/usr/bin/env bash
set -euo pipefail
umask 077
dest="${1:?Usage: scripts/pair.sh NEW_CREDENTIAL_DIRECTORY}"
test ! -e "$dest" || { echo "Refusing to overwrite credentials" >&2; exit 1; }
mkdir -p "$dest/ca" "$dest/server" "$dest/leader"
cat > "$dest/ca/request.cnf" <<EOF
[req]
distinguished_name=dn
x509_extensions=ca_extensions
prompt=no
[dn]
CN=so101-position-ca
[ca_extensions]
basicConstraints=critical,CA:TRUE
keyUsage=critical,keyCertSign,cRLSign
EOF
openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes \
  -keyout "$dest/ca/key.pem" -out "$dest/ca/ca.crt" -days 90 \
  -config "$dest/ca/request.cnf" 2>/dev/null
rm "$dest/ca/request.cnf"
for role in server leader; do
  openssl req -new -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes \
    -keyout "$dest/$role/key.pem" -out "$dest/$role/request.csr" \
    -subj "/CN=so101-$role" 2>/dev/null
  purpose=clientAuth
  if [[ "$role" == server ]]; then purpose=serverAuth; fi
  cat > "$dest/$role/extensions.cnf" <<EOF
basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature
extendedKeyUsage=$purpose
subjectAltName=DNS:so101-$role
EOF
  openssl x509 -req -in "$dest/$role/request.csr" -CA "$dest/ca/ca.crt" \
    -CAkey "$dest/ca/key.pem" -CAcreateserial -out "$dest/$role/cert.pem" \
    -days 30 -extfile "$dest/$role/extensions.cnf" 2>/dev/null
  cp "$dest/ca/ca.crt" "$dest/$role/ca.crt"
  rm "$dest/$role/request.csr" "$dest/$role/extensions.cnf"
done
echo "Paired. Copy only leader/ to the leader computer and server/ to the server. Keep ca/ private."
