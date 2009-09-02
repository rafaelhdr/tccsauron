DECLARE
   var_contador            INTEGER;
   var_estado              VARCHAR2 (4);
BEGIN
   /* Verifica se o usuario digitou o codigo Baan */
   IF (:NEW.codigo_baan IS NOT NULL)
   THEN
      /* Procura o cliente no baan */
      SELECT COUNT (*)
        INTO var_contador
        FROM v_frc_baan_cliente
       WHERE cod_cliente = :NEW.codigo_baan;

      IF (var_contador = 1)
      THEN
         /* Verifica se o cliente nao esta duplicado */
         SELECT COUNT (*)
           INTO var_contador
           FROM frc_clientes_cadastrados
          WHERE frc_clientes_cadastrados.codigo_baan = :NEW.codigo_baan
            AND frc_clientes_cadastrados.id_cliente != :NEW.ID;

         IF (var_contador = 1)
         THEN
            /* O cliente esta duplicado */
            RAISE VALUE_ERROR;
         ELSE
            /* Verifica se o cliente ja foi registrado */
            SELECT COUNT (*)
              INTO var_contador
              FROM frc_clientes_cadastrados
             WHERE frc_clientes_cadastrados.codigo_baan = :NEW.codigo_baan;

            IF (var_contador = 0)
            THEN
               INSERT INTO frc_clientes_cadastrados
                           (id_cliente, codigo_baan
                           )
                    VALUES (:NEW.ID, :NEW.codigo_baan
                           );
            END IF;

            :NEW.estado_cliente := 1;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'codigo_baan';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'codigo_baan', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'razaosocial';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'razaosocial', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'nomefantasia';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'nomefantasia', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'cnpj';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'cnpj', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'nivelrisco';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'nivelrisco', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'limitecredito';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'limitecredito', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'endereco';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'endereco', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'cep';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'cep', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'bairro';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'bairro', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'cidade';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'cidade', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'estado';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'estado', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            SELECT COUNT (*)
              INTO var_contador
              FROM odf_locked_attributes
             WHERE odf_pk = :NEW.ID
               AND object_code = 'frc_cliente'
               AND attribute_code = 'pais';

            IF (var_contador = 0)
            THEN
               INSERT INTO odf_locked_attributes
                           (ID, object_code,
                            odf_pk, attribute_code, created_date,
                            created_by, last_updated_date, last_updated_by
                           )
                    VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                            :NEW.ID, 'pais', SYSDATE,
                            1, SYSDATE, 1
                           );
            END IF;

            /* O cliente esta cadastrado no baan */
            SELECT
		substr(Razao_Social, 0, 132),
		substr(Nome_Fantasia, 0, 40),
		substr(cnpj, 0, 60),
		substr(endereco, 0, 132),
		substr(cep, 0, 20),
		substr(bairro, 0, 60),
		substr(cidade, 0, 60),
			substr(estado, 0, 2),
		substr(pais, 0, 60),
		limite_credito,
		nivel_risco,
		Pai_Financeiro,
		Razao_Social_Pai
	INTO
		:NEW.razaosocial,
		:NEW.nomefantasia,
		:NEW.cnpj,
		:NEW.endereco,
		:NEW.cep,
		:NEW.bairro,
		:NEW.cidade,
		var_estado,
		:NEW.pais,
		:NEW.limitecredito,
		:NEW.nivelrisco,
		:NEW.codigo_baan_pai,
		:NEW.paifinanceiro
	FROM
		v_frc_baan_cliente
	WHERE
		cod_cliente = :NEW.codigo_baan;

            /* Verifica se o estado esta cadastrado */
            SELECT COUNT (*)
              INTO var_contador
              FROM cmn_lookups lookups, cmn_captions_nls captions
             WHERE captions.pk_id = lookups.ID
               AND captions.language_code = 'en'
               AND captions.table_name = 'CMN_LOOKUPS'
               AND lookups.lookup_type = 'SPEC_UF'
               AND captions.NAME like var_estado;

            IF (var_contador = 1)
            THEN
               SELECT lookups.lookup_enum
                 INTO :NEW.estado
                 FROM cmn_lookups lookups, cmn_captions_nls captions
                WHERE captions.pk_id = lookups.ID
                  AND captions.language_code = 'en'
                  AND captions.table_name = 'CMN_LOOKUPS'
                  AND lookups.lookup_type = 'SPEC_UF'
                  AND captions.NAME like var_estado;
            ELSE
               :NEW.estado := NULL;
            END IF;

            /* Verifica se o pai financeiro esta criado e faz o link com ele */
            IF (    :NEW.codigo_baan_pai IS NOT NULL
                AND :NEW.paifinanceiro IS NOT NULL
               )
            THEN
               frc_pai_financeiro_criar (:NEW.codigo_baan_pai,
                                         :NEW.paifinanceiro,
                                         :NEW.id_paifinanceiro
                                        );
            ELSE
               frc_pai_financeiro_criar (:NEW.codigo_baan,
                                         :NEW.razaosocial,
                                         :NEW.id_paifinanceiro
                                        );
               :NEW.paifinanceiro := :NEW.razaosocial;
	       :NEW.codigo_baan_pai := :NEW.codigo_baan;
            END IF;

            /* Verifica se o cliente esta no Visionarium */
            SELECT COUNT (*)
              INTO var_contador
              FROM v_frc_abm_balancos
             WHERE cnpj = :NEW.cnpj;

            IF (var_contador > 0)
            THEN
               /* Obtem os dados do ultimo balanco */
               SELECT data_balanco, faturamento_bruto,
                      faturamento_liquido, patrimonio_liquido,
                      lucro_prejuizo, capital_de_giro,
                      margem_liquida, ro_fl
                 INTO :NEW.data_balanco, :NEW.faturamento_bruto,
                      :NEW.faturamento_liquido, :NEW.patrimonio_liquido,
                      :NEW.lucro_prejuizo, :NEW.capital_giro,
                      :NEW.margem_liquida, :NEW.ro_fl
                 FROM (SELECT   mes_balanco || '/'
                                || ano_balanco data_balanco,
                                faturamento_bruto, faturamento_liquido,
                                patrimonio_liquido, lucro_prejuizo,
                                capital_de_giro, margem_liquida, ro_fl
                           FROM v_frc_abm_balancos
                          WHERE cnpj = :NEW.cnpj
                       ORDER BY ano_balanco DESC, mes_balanco DESC)
                WHERE ROWNUM = 1;
            ELSE
               /* Limpa os dados de balanco */
               :NEW.data_balanco := '';
               :NEW.faturamento_bruto := '';
               :NEW.faturamento_liquido := '';
               :NEW.patrimonio_liquido := '';
               :NEW.lucro_prejuizo := '';
               :NEW.capital_giro := '';
               :NEW.margem_liquida := '';
               :NEW.ro_fl := '';
            END IF;

            /* Apaga a lista dos acionistas do cliente */
            FOR acionista IN (SELECT odf_ca_frc_acionista.ID
                                FROM odf_ca_frc_acionista
                               WHERE odf_ca_frc_acionista.odf_parent_id =
                                                                       :NEW.ID)
            LOOP
               frc_acionista_remover (acionista.ID);
            END LOOP;

            /* Cria a lista dos acionistas do cliente */
            FOR acionista IN (SELECT nome_acionista,
                                     porcentagem_preferenciais,
                                     porcentagem_ordinarias
                                FROM v_frc_abm_acionistas
                               WHERE cnpj = :NEW.cnpj)
            LOOP
               frc_acionista_criar (:NEW.ID,
                                    acionista.nome_acionista,
                                    acionista.porcentagem_ordinarias,
                                    acionista.porcentagem_preferenciais
                                   );
            END LOOP;

            /* Se o cliente foi de prospeccao entao altera o lock do objeto de edicao do cliente */
            SELECT COUNT (*)
              INTO var_contador
              FROM odf_ca_frc_ed_cliente
             WHERE odf_parent_id = :NEW.ID;

            IF (var_contador = 1)
            THEN
               SELECT COUNT (*)
                 INTO var_contador
                 FROM odf_ca_frc_ed_cliente, odf_locked_attributes
                WHERE odf_ca_frc_ed_cliente.ID = odf_locked_attributes.odf_pk
                  AND odf_locked_attributes.object_code = 'frc_ed_cliente'
                  AND odf_locked_attributes.attribute_code = 'codigo_baan'
                  AND odf_ca_frc_ed_cliente.odf_parent_id = :NEW.ID;

               IF (var_contador = 1)
               THEN
                  DELETE FROM odf_locked_attributes
                        WHERE attribute_code = 'codigo_baan'
                          AND object_code = 'frc_ed_cliente'
                          AND odf_pk IN (SELECT ID
                                           FROM odf_ca_frc_ed_cliente
                                          WHERE odf_parent_id = :NEW.ID);

                  INSERT INTO odf_locked_attributes
                              (ID,
                               object_code,
                               odf_pk,
                               attribute_code, created_date, created_by,
                               last_updated_date, last_updated_by
                              )
                       VALUES (odf_locked_attributes_s1.NEXTVAL,
                               'frc_ed_cliente',
                               (SELECT ID
                                  FROM odf_ca_frc_ed_cliente
                                 WHERE odf_parent_id = :NEW.ID),
                               'razaosocial', SYSDATE, 1,
                               SYSDATE, 1
                              );
               END IF;
            END IF;
         END IF;
      ELSE
      
      
         /* O cliente nao esta cadastrado no baan */
         RAISE VALUE_ERROR;
      END IF;
   ELSE
      IF (   :NEW.razaosocial IS NULL
          OR :NEW.razaosocial = 'Digite a razão social'
         )
      THEN
         /* O cliente nao esta cadastrado no baan */
         RAISE VALUE_ERROR;
      ELSE
         :NEW.id_paifinanceiro := 0;
         :NEW.estado_cliente := 2;
         /* Limpa os dados de balanco */
         :NEW.data_balanco := '';
         :NEW.faturamento_bruto := '';
         :NEW.faturamento_liquido := '';
         :NEW.patrimonio_liquido := '';
         :NEW.lucro_prejuizo := '';
         :NEW.capital_giro := '';
         :NEW.margem_liquida := '';
         :NEW.ro_fl := '';

         SELECT COUNT (*)
           INTO var_contador
           FROM odf_locked_attributes
          WHERE odf_pk = :NEW.ID
            AND object_code = 'frc_cliente'
            AND attribute_code = 'razaosocial';

         IF (var_contador = 0)
         THEN
            INSERT INTO odf_locked_attributes
                        (ID, object_code,
                         odf_pk, attribute_code, created_date, created_by,
                         last_updated_date, last_updated_by
                        )
                 VALUES (odf_locked_attributes_s1.NEXTVAL, 'frc_cliente',
                         :NEW.ID, 'razaosocial', SYSDATE, 1,
                         SYSDATE, 1
                        );
         END IF;

         frc_ed_cliente_criar (:NEW.ID, :NEW.razaosocial);
      END IF;
   END IF;

   :NEW.code := :NEW.ID;
   :NEW.NAME := :NEW.razaosocial;

   UPDATE odf_ca_frc_ed_cliente
      SET razaosocial = :NEW.razaosocial,
          codigo_baan = :NEW.codigo_baan
    WHERE odf_parent_id = :NEW.ID;
END;
